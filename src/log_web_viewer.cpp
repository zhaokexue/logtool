#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <filesystem>
#include <cmath>
#include <vector>
#include <limits>
#include <stdexcept>
#include <sstream>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>

#include "log_io.hpp"
#include "index.hpp"
#include "codec.hpp"
#include "log_types.hpp"

// ---------------- trajectory ----------------
// We build a pose timeline once (startup) and serve trajectory queries from it.
struct PoseRec {
    uint64_t ts_ns{0};
    float x{0.f};
    float y{0.f};
    float yaw{0.f};
};

static std::vector<PoseRec> buildPoseTimeline(const IndexDB& db, LogReader& reader) {
    std::vector<PoseRec> poses;
    auto it = db.by_type.find(TYPE_POSE);
    if (it == db.by_type.end() || it->second.empty()) return poses;

    poses.reserve(it->second.size());
    for (const auto& item : it->second) {
        Record r = reader.readAt(item.offset);
        Pose2D p = parseRecordPayload<Pose2D>(r);
        poses.push_back(PoseRec{r.timestamp, p.x, p.y, p.angle});
    }
    std::sort(poses.begin(), poses.end(), [](const PoseRec& a, const PoseRec& b){
        return a.ts_ns < b.ts_ns;
    });
    return poses;
}

static size_t lastIndexLE(const std::vector<PoseRec>& poses, uint64_t ts_ns) {
    if (poses.empty()) return 0;
    auto it = std::upper_bound(
        poses.begin(), poses.end(), ts_ns,
        [](uint64_t v, const PoseRec& p){ return v < p.ts_ns; }
    );
    if (it == poses.begin()) return 0;
    return static_cast<size_t>((it - poses.begin()) - 1);
}

static std::vector<float> buildTrajPrefixXY(const std::vector<PoseRec>& poses,
                                            uint64_t target_ts,
                                            uint64_t step_ms,
                                            size_t max_points) {
    std::vector<float> out;
    if (poses.empty()) return out;
    if (step_ms == 0) step_ms = 50;
    if (max_points == 0) max_points = 8000;

    size_t end_idx = lastIndexLE(poses, target_ts);
    const uint64_t step_ns = step_ms * 1000000ULL;
    const uint64_t t0 = poses.front().ts_ns;
    const uint64_t t_end = poses[end_idx].ts_ns;

    out.reserve(std::min(max_points, end_idx + 1) * 2);

    // Monotonic walk through poses for O(n) sampling.
    size_t cur = 0;
    float lastx = std::numeric_limits<float>::quiet_NaN();
    float lasty = std::numeric_limits<float>::quiet_NaN();

    for (uint64_t t = t0; t <= t_end && (out.size() / 2) < max_points; t += step_ns) {
        while (cur + 1 <= end_idx && poses[cur + 1].ts_ns <= t) cur++;
        const auto& p = poses[cur];

        // De-dup very small movements to keep the polyline light.
        if (std::isfinite(lastx)) {
            const float dx = p.x - lastx;
            const float dy = p.y - lasty;
            if ((dx*dx + dy*dy) < 1e-4f) continue; // < 1cm
        }

        out.push_back(p.x);
        out.push_back(p.y);
        lastx = p.x;
        lasty = p.y;
    }

    // Ensure the end point is included.
    const auto& pe = poses[end_idx];
    if (out.size() < 2 || out[out.size() - 2] != pe.x || out[out.size() - 1] != pe.y) {
        if ((out.size() / 2) < max_points) {
            out.push_back(pe.x);
            out.push_back(pe.y);
        }
    }

    return out;
}

// ---------------- helpers ----------------
static bool ends_with(const std::string& s, const std::string& suffix) {
    if (s.size() < suffix.size()) return false;
    return std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

static std::string readFileBinary(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return "";
    return std::string((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());
}

static std::string contentType(const std::string& path) {
    if (ends_with(path, ".html")) return "text/html; charset=utf-8";
    if (ends_with(path, ".js"))   return "application/javascript; charset=utf-8";
    if (ends_with(path, ".css"))  return "text/css; charset=utf-8";
    if (ends_with(path, ".json")) return "application/json; charset=utf-8";
    if (ends_with(path, ".png"))  return "image/png";
    return "application/octet-stream";
}

static bool fileExists(const std::string& p) {
    std::error_code ec;
    return std::filesystem::exists(p, ec);
}

static std::string defaultIdxPath(const std::string& log_path) {
    std::filesystem::path p(log_path);
    p.replace_extension(".idx");
    return p.string();
}

// Build an in-memory index by scanning the log file sequentially.
// Offsets always point to RecordHeader, consistent with logtool's writeRecord().
static IndexDB buildIndexByScan(const std::string& log_path) {
    uint64_t fsz = static_cast<uint64_t>(std::filesystem::file_size(log_path));

    std::ifstream ifs(log_path, std::ios::binary);
    if (!ifs) throw std::runtime_error("cannot open log: " + log_path);

    FileHeader fh{};
    ifs.read(reinterpret_cast<char*>(&fh), sizeof(fh));
    if (!ifs) throw std::runtime_error("read FileHeader failed");
    if (fh.magic != 0x31474F4C) throw std::runtime_error("bad log magic");
    if (fh.version != 1) throw std::runtime_error("unsupported log version");

    IndexDB db;
    uint64_t offset = sizeof(FileHeader);
    while (true) {
        if (offset + sizeof(RecordHeader) > fsz) break;

        RecordHeader h{};
        ifs.read(reinterpret_cast<char*>(&h), sizeof(h));
        if (!ifs) break;

        uint64_t payload_off = offset + sizeof(RecordHeader);
        uint64_t end_off = payload_off + h.length;
        if (end_off > fsz) {
            throw std::runtime_error("log corrupted (length overrun) at offset=" + std::to_string(offset));
        }

        db.all.push_back(IndexItem{h.timestamp_ns, offset, h.type});

        ifs.seekg(static_cast<std::streamoff>(h.length), std::ios::cur);
        if (!ifs) throw std::runtime_error("scan skip payload failed");
        offset = end_off;
    }

    std::sort(db.all.begin(), db.all.end(),
              [](const IndexItem& a, const IndexItem& b){ return a.timestamp_ns < b.timestamp_ns; });
    db.buildTypeViews();
    return db;
}

static IndexDB loadOrBuildIndex(const std::string& log_path, const std::string& idx_path, bool force_rebuild) {
    if (!force_rebuild && fileExists(idx_path)) {
        try {
            return loadIndex(idx_path);
        } catch (...) {
            // fallback to rebuild
        }
    }
    IndexDB db = buildIndexByScan(log_path);
    try { saveIndex(idx_path, db); } catch (...) {}
    return db;
}

static std::string makeMetaJson(const IndexDB& db) {
    uint64_t t0 = db.all.empty() ? 0 : db.all.front().timestamp_ns;
    uint64_t t1 = db.all.empty() ? 0 : db.all.back().timestamp_ns;
    double dur = (t1 > t0) ? double(t1 - t0) / 1e9 : 0.0;

    const bool has_map  = db.by_type.count(TYPE_MAP)  && !db.by_type.at(TYPE_MAP).empty();
    const bool has_imu  = db.by_type.count(TYPE_IMU)  && !db.by_type.at(TYPE_IMU).empty();
    const bool has_odom = db.by_type.count(TYPE_ODOM) && !db.by_type.at(TYPE_ODOM).empty();
    const bool has_slip = db.by_type.count(TYPE_SLIP) && !db.by_type.at(TYPE_SLIP).empty();
    const bool has_state= db.by_type.count(TYPE_STATE)&& !db.by_type.at(TYPE_STATE).empty();

    uint64_t map_latest_ts = 0;
    if (has_map) map_latest_ts = db.by_type.at(TYPE_MAP).back().timestamp_ns;

    return "{" 
        "\"t0_ns\":" + std::to_string(t0) + "," 
        "\"t1_ns\":" + std::to_string(t1) + "," 
        "\"duration_sec\":" + std::to_string(dur) + "," 
        "\"has_map\":" + std::string(has_map ? "true" : "false") + "," 
        "\"has_imu\":" + std::string(has_imu ? "true" : "false") + "," 
        "\"has_odom\":" + std::string(has_odom ? "true" : "false") + "," 
        "\"has_slip\":" + std::string(has_slip ? "true" : "false") + "," 
        "\"has_state\":" + std::string(has_state ? "true" : "false") + "," 
        "\"map_latest_ts\":" + std::to_string(map_latest_ts) +
    "}";
}

static std::string b64encode(const uint8_t* data, size_t len) {
    static const char* tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    for (size_t i = 0; i < len; i += 3) {
        uint32_t v = (uint32_t(data[i]) << 16);
        if (i + 1 < len) v |= (uint32_t(data[i + 1]) << 8);
        if (i + 2 < len) v |= (uint32_t(data[i + 2]));

        out.push_back(tbl[(v >> 18) & 63]);
        out.push_back(tbl[(v >> 12) & 63]);
        if (i + 1 < len) out.push_back(tbl[(v >> 6) & 63]); else out.push_back('=');
        if (i + 2 < len) out.push_back(tbl[v & 63]); else out.push_back('=');
    }
    return out;
}

static std::string stateToText(uint32_t s) {
    // Map to logtool's RobotState enum values.
    switch (s) {
        case 0: return "selfcheck";
        case 1: return "checkdock";
        case 2: return "findwall";
        case 3: return "followwall";
        case 4: return "coverage";
        case 5: return "gohome";
        case 6: return "end";
        default: return std::to_string(s);
    }
}

// Build /api/frame response compatible with existing web/app.js
static std::string makeFrameJson(uint64_t target_ts, const IndexDB& db, LogReader& reader) {
    // Pose is the primary timeline for UI. If missing, fallback to scan.
    const IndexItem* ip = db.nearest(TYPE_POSE, target_ts);
    if (!ip) return "{\"error\":\"no pose\"}";

    Record rp = reader.readAt(ip->offset);
    Pose2D pose = parseRecordPayload<Pose2D>(rp);
    const uint64_t pose_ts = rp.timestamp;

    // scan aligned to pose_ts for stable visualization
    const IndexItem* is = db.nearest(TYPE_SCAN, pose_ts);
    LaserScan scan;
    uint64_t scan_ts = 0;
    bool has_scan = false;
    if (is) {
        Record rs = reader.readAt(is->offset);
        scan = parseRecordPayload<LaserScan>(rs);
        scan_ts = rs.timestamp;
        has_scan = true;
    }

    // optional telemetry
    bool has_imu = false, has_odom = false, has_slip = false, has_state = false;
    Imu imu{};
    Odom odom{};
    Slip slip{};
    RobotState state = RobotState::selfcheck;
    uint64_t map_ts = 0;

    if (const IndexItem* ii = db.nearest(TYPE_IMU, pose_ts)) {
        Record ri = reader.readAt(ii->offset);
        imu = parseRecordPayload<Imu>(ri);
        has_imu = true;
    }
    if (const IndexItem* io = db.nearest(TYPE_ODOM, pose_ts)) {
        Record ro = reader.readAt(io->offset);
        odom = parseRecordPayload<Odom>(ro);
        has_odom = true;
    }
    if (const IndexItem* isl = db.nearest(TYPE_SLIP, pose_ts)) {
        Record rsl = reader.readAt(isl->offset);
        slip = parseRecordPayload<Slip>(rsl);
        has_slip = true;
    }
    if (const IndexItem* ist = db.nearest(TYPE_STATE, pose_ts)) {
        Record rst = reader.readAt(ist->offset);
        state = parseRecordPayload<RobotState>(rst);
        has_state = true;
    }
    if (const IndexItem* im = db.nearest(TYPE_MAP, pose_ts)) {
        map_ts = im->timestamp_ns;
    }

    const uint64_t t0 = db.all.empty() ? 0 : db.all.front().timestamp_ns;
    const double rel_sec = (pose_ts > t0) ? double(pose_ts - t0) / 1e9 : 0.0;

    std::string j;
    j.reserve(96 * 1024);
    j += "{";
    j += "\"time_text\":\"t0+" + std::to_string(rel_sec) + "s\",";
    j += "\"pose_ts\":" + std::to_string(pose_ts) + ",";
    j += "\"scan_ts\":" + std::to_string(scan_ts) + ",";
    j += "\"map_ts\":" + std::to_string(map_ts) + ",";
    j += "\"pose\":{";
    j += "\"x\":" + std::to_string(pose.x) + ",";
    j += "\"y\":" + std::to_string(pose.y) + ",";
    j += "\"yaw\":" + std::to_string(pose.angle) + "},";

    if (has_imu) {
        j += "\"imu\":{";
        j += "\"pitch\":" + std::to_string(imu.pitch) + ",";
        j += "\"roll\":" + std::to_string(imu.roll) + ",";
        j += "\"yaw\":" + std::to_string(imu.yaw) + "},";
    } else {
        j += "\"imu\":null,";
    }

    if (has_odom) {
        // UI shows one odom; default to fuse_pose.
        j += "\"odom\":{";
        j += "\"x\":" + std::to_string(odom.fuse_pose.x) + ",";
        j += "\"y\":" + std::to_string(odom.fuse_pose.y) + ",";
        j += "\"yaw\":" + std::to_string(odom.fuse_pose.angle) + "},";
    } else {
        j += "\"odom\":null,";
    }

    if (has_slip) {
        int slip_any = (slip.line_slip || slip.rotate_slip) ? 1 : 0;
        j += "\"slip\":" + std::to_string(slip_any) + ",";
    } else {
        j += "\"slip\":null,";
    }

    if (has_state) {
        j += "\"state\":\"" + stateToText(static_cast<uint32_t>(state)) + "\",";
    } else {
        j += "\"state\":null,";
    }

    // points in robot-local frame
    j += "\"points\":[";
    if (has_scan) {
        bool first = true;
        for (size_t i = 0; i < scan.beams.size(); ++i) {
            float r = scan.beams[i];
            float ang = scan.angle_min + float(i) * scan.angle_increment;
            float lx = r * std::cos(ang);
            float ly = r * std::sin(ang);
            if (!first) j += ",";
            first = false;
            j += "[" + std::to_string(lx) + "," + std::to_string(ly) + "]";
        }
    }
    j += "]";
    j += "}";
    return j;
}

static std::string makeMapJson(uint64_t target_ts, const IndexDB& db, LogReader& reader) {
    const IndexItem* im = db.nearest(TYPE_MAP, target_ts);
    if (!im) return "{\"error\":\"no map\"}";
    Record rm = reader.readAt(im->offset);
    GridMap map = parseRecordPayload<GridMap>(rm);

    // GridMap::data is std::vector<int8_t> (signed). We base64 encode raw bytes.
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(map.data.data());
    std::string b64 = b64encode(raw, map.data.size());

    std::string j;
    j.reserve(b64.size() + 512);
    j += "{";
    j += "\"map_ts\":" + std::to_string(rm.timestamp) + ",";
    j += "\"w\":" + std::to_string(map.width) + ",";
    j += "\"h\":" + std::to_string(map.height) + ",";
    j += "\"res\":" + std::to_string(map.resolution) + ",";
    j += "\"ox\":" + std::to_string(map.origin_x) + ",";
    j += "\"oy\":" + std::to_string(map.origin_y) + ",";
    j += "\"data_b64\":\"" + b64 + "\"";
    j += "}";
    return j;
}

static std::string makeTrajJson(uint64_t target_ts,
                                uint64_t step_ms,
                                size_t max_points,
                                const std::vector<PoseRec>& poses) {
    if (poses.empty()) return "{\"error\":\"no pose\"}";
    if (step_ms == 0) step_ms = 50;
    if (max_points == 0) max_points = 8000;

    std::vector<float> xy = buildTrajPrefixXY(poses, target_ts, step_ms, max_points);
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(xy.data());
    const size_t raw_len = xy.size() * sizeof(float);
    std::string b64 = b64encode(raw, raw_len);

    std::string j;
    j.reserve(b64.size() + 256);
    j += "{";
    j += "\"t0_ns\":" + std::to_string(poses.front().ts_ns) + ",";
    j += "\"t1_ns\":" + std::to_string(target_ts) + ",";
    j += "\"step_ms\":" + std::to_string(step_ms) + ",";
    j += "\"count\":" + std::to_string(xy.size() / 2) + ",";
    j += "\"xy_f32_b64\":\"" + b64 + "\"";
    j += "}";
    return j;
}

// -------- minimal HTTP server --------
struct HttpRequest {
    std::string method;
    std::string target;
};

static bool readHttpRequest(int fd, HttpRequest& out) {
    std::string buf;
    buf.reserve(4096);
    char tmp[2048];

    // read until header end or too large
    while (buf.find("\r\n\r\n") == std::string::npos) {
        ssize_t n = ::recv(fd, tmp, sizeof(tmp), 0);
        if (n <= 0) return false;
        buf.append(tmp, tmp + n);
        if (buf.size() > 64 * 1024) return false;
    }

    // parse request line
    auto line_end = buf.find("\r\n");
    if (line_end == std::string::npos) return false;
    std::string line = buf.substr(0, line_end);

    std::istringstream iss(line);
    iss >> out.method >> out.target;
    return !out.method.empty() && !out.target.empty();
}

static void sendHttpResponse(int fd, int status, const std::string& content_type, const std::string& body) {
    std::string status_text = "OK";
    if (status == 404) status_text = "Not Found";
    else if (status == 400) status_text = "Bad Request";
    else if (status == 500) status_text = "Internal Server Error";

    std::ostringstream oss;
    oss << "HTTP/1.1 " << status << " " << status_text << "\r\n";
    oss << "Content-Type: " << content_type << "\r\n";
    oss << "Content-Length: " << body.size() << "\r\n";
    oss << "Connection: close\r\n";
    oss << "\r\n";
    std::string header = oss.str();

    ::send(fd, header.data(), header.size(), 0);
    if (!body.empty()) ::send(fd, body.data(), body.size(), 0);
}

static std::string getQueryParamU64(const std::string& target, const std::string& key, uint64_t def=0) {
    auto qpos = target.find('?');
    if (qpos == std::string::npos) return "";
    std::string q = target.substr(qpos + 1);
    auto kpos = q.find(key + "=");
    if (kpos == std::string::npos) return "";
    kpos += key.size() + 1;
    auto end = q.find('&', kpos);
    std::string val = (end == std::string::npos) ? q.substr(kpos) : q.substr(kpos, end - kpos);
    return val;
}

int main(int argc, char** argv) {
    signal(SIGPIPE, SIG_IGN);   // 关键：避免 Broken pipe 直接杀进程
    try {
        std::string log_path = "data/fake_cleaning.log";
        std::string web_root = "web";
        std::string idx_path = "";
        int port = 8080;
        bool rebuild_idx = false;

        for (int i = 1; i + 1 < argc; ++i) {
            std::string k = argv[i];
            if (k == "--log")  log_path = argv[i + 1];
            if (k == "--web")  web_root = argv[i + 1];
            if (k == "--idx")  idx_path = argv[i + 1];
            if (k == "--port") port = std::atoi(argv[i + 1]);
        }
        for (int i = 1; i < argc; ++i) {
            if (std::string(argv[i]) == "--rebuild_idx") rebuild_idx = true;
        }
        if (idx_path.empty()) idx_path = defaultIdxPath(log_path);

        LogReader reader(log_path);
        IndexDB db = loadOrBuildIndex(log_path, idx_path, rebuild_idx);

        // Build pose timeline for trajectory queries (seek/drag correctness + lightweight rendering).
        std::vector<PoseRec> poses = buildPoseTimeline(db, reader);

        double dur = (db.all.empty() ? 0.0 : double(db.all.back().timestamp_ns - db.all.front().timestamp_ns) / 1e9);
        std::cout << "Index ready. items=" << db.all.size() << " duration=" << dur << " sec\n";
        std::cout << "Pose timeline ready. poses=" << poses.size() << "\n";
        std::cout << "Serving http://127.0.0.1:" << port << "\n";

        int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) throw std::runtime_error("socket() failed");

        int yes = 1;
        ::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (::bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error("bind() failed");
        }
        if (::listen(server_fd, 16) < 0) throw std::runtime_error("listen() failed");

        while (true) {
            sockaddr_in caddr{};
            socklen_t clen = sizeof(caddr);
            int client_fd = ::accept(server_fd, reinterpret_cast<sockaddr*>(&caddr), &clen);
            if (client_fd < 0) continue;

            HttpRequest req{};
            if (!readHttpRequest(client_fd, req)) {
                sendHttpResponse(client_fd, 400, "text/plain; charset=utf-8", "Bad Request");
                ::close(client_fd);
                continue;
            }

            std::string target = req.target;

            // API
            if (target == "/api/meta") {
                sendHttpResponse(client_fd, 200, "application/json; charset=utf-8", makeMetaJson(db));
                ::close(client_fd);
                continue;
            }

            if (target.rfind("/api/frame", 0) == 0) {
                uint64_t ts = 0;
                std::string v = getQueryParamU64(target, "ts_ns");
                if (!v.empty()) ts = static_cast<uint64_t>(std::strtoull(v.c_str(), nullptr, 10));
                std::string body = makeFrameJson(ts, db, reader);
                sendHttpResponse(client_fd, 200, "application/json; charset=utf-8", body);
                ::close(client_fd);
                continue;
            }

            if (target.rfind("/api/map", 0) == 0) {
                uint64_t ts = 0;
                std::string v = getQueryParamU64(target, "ts_ns");
                if (!v.empty()) ts = static_cast<uint64_t>(std::strtoull(v.c_str(), nullptr, 10));
                std::string body = makeMapJson(ts, db, reader);
                sendHttpResponse(client_fd, 200, "application/json; charset=utf-8", body);
                ::close(client_fd);
                continue;
            }

            if (target.rfind("/api/traj", 0) == 0) {
                uint64_t ts = 0;
                uint64_t step_ms = 50;
                size_t max_points = 8000;

                std::string v = getQueryParamU64(target, "ts_ns");
                if (!v.empty()) ts = static_cast<uint64_t>(std::strtoull(v.c_str(), nullptr, 10));
                v = getQueryParamU64(target, "step_ms");
                if (!v.empty()) step_ms = static_cast<uint64_t>(std::strtoull(v.c_str(), nullptr, 10));
                v = getQueryParamU64(target, "max_points");
                if (!v.empty()) max_points = static_cast<size_t>(std::strtoull(v.c_str(), nullptr, 10));

                std::string body = makeTrajJson(ts, step_ms, max_points, poses);
                sendHttpResponse(client_fd, 200, "application/json; charset=utf-8", body);
                ::close(client_fd);
                continue;
            }

            // Static
            std::string path = target;
            if (path == "/") path = "/index.html";
            if (path.find("..") != std::string::npos) {
                sendHttpResponse(client_fd, 400, "text/plain; charset=utf-8", "Bad Request");
                ::close(client_fd);
                continue;
            }

            std::string full = web_root + path;
            std::string body = readFileBinary(full);
            if (body.empty()) {
                sendHttpResponse(client_fd, 404, "text/plain; charset=utf-8", "404 Not Found");
                ::close(client_fd);
                continue;
            }
            sendHttpResponse(client_fd, 200, contentType(full), body);
            ::close(client_fd);
        }

    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    }
}
