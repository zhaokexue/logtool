#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <filesystem>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <sstream>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "log_io.hpp"
#include "index.hpp"
#include "codec.hpp"
#include "log_types.hpp"

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

    return "{"
        "\"t0_ns\":" + std::to_string(t0) + ","
        "\"t1_ns\":" + std::to_string(t1) + ","
        "\"duration_sec\":" + std::to_string(dur) +
    "}";
}

// Build /api/frame response compatible with existing web/app.js
static std::string makeFrameJson(uint64_t target_ts, const IndexDB& db, LogReader& reader) {
    const IndexItem* is = db.nearest(TYPE_SCAN, target_ts);
    if (!is) return "{\"error\":\"no scan\"}";

    Record rs = reader.readAt(is->offset);
    LaserScan scan = parseRecordPayload<LaserScan>(rs);

    const IndexItem* ip = db.nearest(TYPE_POSE, rs.timestamp);
    if (!ip) return "{\"error\":\"no pose\"}";

    Record rp = reader.readAt(ip->offset);
    Pose2D pose = parseRecordPayload<Pose2D>(rp);

    std::string j;
    j.reserve(64 * 1024);

    j += "{";
    j += "\"pose_ts\":" + std::to_string(rp.timestamp) + ",";
    j += "\"scan_ts\":" + std::to_string(rs.timestamp) + ",";
    j += "\"pose\":{";
    j += "\"x\":" + std::to_string(pose.x) + ",";
    j += "\"y\":" + std::to_string(pose.y) + ",";
    j += "\"yaw\":" + std::to_string(pose.angle) + "},";
    j += "\"points\":[";

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

    j += "]";
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

        double dur = (db.all.empty() ? 0.0 : double(db.all.back().timestamp_ns - db.all.front().timestamp_ns) / 1e9);
        std::cout << "Index ready. items=" << db.all.size() << " duration=" << dur << " sec\n";
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
