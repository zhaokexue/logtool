#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <random>
#include <cmath>

#include "log_types.hpp"
#include "codec.hpp"
#include "log_io.hpp"
#include "index.hpp"

// =========================
// CSV trajectory loader
// Expected columns: timestamp_ns,x_m,y_m,angle_rad
// =========================
struct PoseSample {
    uint64_t ts{0};
    float x{0}, y{0}, a{0};
};

inline std::vector<PoseSample> loadTrajectoryCSV(const std::string& csv_path) {
    std::ifstream ifs(csv_path);
    if (!ifs) throw std::runtime_error("loadTrajectoryCSV: cannot open " + csv_path);

    std::string line;
    std::vector<PoseSample> out;

    // header
    if (!std::getline(ifs, line)) throw std::runtime_error("loadTrajectoryCSV: empty file");

    while (std::getline(ifs, line)) {
        if (line.empty()) continue;

        // split by comma
        size_t p = 0;
        auto nextTok = [&](size_t& pos) -> std::string {
            size_t q = line.find(',', pos);
            std::string tok = (q == std::string::npos) ? line.substr(pos) : line.substr(pos, q - pos);
            pos = (q == std::string::npos) ? std::string::npos : q + 1;
            return tok;
        };

        std::string s_ts = nextTok(p);
        std::string s_x  = (p==std::string::npos) ? "" : nextTok(p);
        std::string s_y  = (p==std::string::npos) ? "" : nextTok(p);
        std::string s_a  = (p==std::string::npos) ? "" : nextTok(p);

        if (s_ts.empty() || s_x.empty() || s_y.empty() || s_a.empty()) continue;

        PoseSample ps;
        ps.ts = static_cast<uint64_t>(std::stoull(s_ts));
        ps.x  = std::stof(s_x);
        ps.y  = std::stof(s_y);
        ps.a  = std::stof(s_a);
        out.push_back(ps);
    }

    if (out.size() < 2) throw std::runtime_error("loadTrajectoryCSV: too few samples");
    std::sort(out.begin(), out.end(), [](const PoseSample& a, const PoseSample& b){ return a.ts < b.ts; });
    return out;
}

inline PoseSample samplePoseNearest(const std::vector<PoseSample>& traj, uint64_t target_ts) {
    auto it = std::lower_bound(traj.begin(), traj.end(), target_ts,
                               [](const PoseSample& a, uint64_t ts){ return a.ts < ts; });
    if (it == traj.begin()) return *it;
    if (it == traj.end()) return traj.back();
    const auto& r = *it;
    const auto& l = *(it - 1);
    return (target_ts - l.ts <= r.ts - target_ts) ? l : r;
}

// =========================
// Minimal NPY loader: int16, 2D, C-order
// Supports numpy np.save(int16[H,W]) output from our script.
// =========================
struct NpyArrayI16 {
    int32_t H{0}, W{0};
    std::vector<int16_t> data;
};

inline std::string readExact(std::ifstream& ifs, size_t n) {
    std::string s(n, '\0');
    ifs.read(&s[0], static_cast<std::streamsize>(n));
    if (!ifs) throw std::runtime_error("NPY: read failed");
    return s;
}

inline NpyArrayI16 loadNpyInt16_2D(const std::string& npy_path) {
    std::ifstream ifs(npy_path, std::ios::binary);
    if (!ifs) throw std::runtime_error("loadNpyInt16_2D: cannot open " + npy_path);

    // magic
    auto magic = readExact(ifs, 6);
    if (magic != "\x93NUMPY") throw std::runtime_error("NPY: bad magic");

    uint8_t ver_major = 0, ver_minor = 0;
    ifs.read(reinterpret_cast<char*>(&ver_major), 1);
    ifs.read(reinterpret_cast<char*>(&ver_minor), 1);
    if (!ifs) throw std::runtime_error("NPY: read version failed");

    uint32_t header_len = 0;
    if (ver_major == 1) {
        uint16_t hl16 = 0;
        ifs.read(reinterpret_cast<char*>(&hl16), 2);
        header_len = hl16;
    } else if (ver_major == 2) {
        uint32_t hl32 = 0;
        ifs.read(reinterpret_cast<char*>(&hl32), 4);
        header_len = hl32;
    } else {
        throw std::runtime_error("NPY: unsupported version");
    }
    if (!ifs) throw std::runtime_error("NPY: read header length failed");

    std::string header = readExact(ifs, header_len);

    if (header.find("<i2") == std::string::npos)
        throw std::runtime_error("NPY: only supports little-endian int16 (<i2)");
    if (header.find("fortran_order") == std::string::npos)
        throw std::runtime_error("NPY: missing fortran_order");
    if (header.find("fortran_order") != std::string::npos && header.find("True") != std::string::npos)
        throw std::runtime_error("NPY: fortran_order=True not supported");

    auto pshape = header.find("shape");
    if (pshape == std::string::npos) throw std::runtime_error("NPY: missing shape");
    auto lp = header.find('(', pshape);
    auto rp = header.find(')', pshape);
    if (lp == std::string::npos || rp == std::string::npos || rp <= lp)
        throw std::runtime_error("NPY: bad shape");

    std::string inside = header.substr(lp + 1, rp - lp - 1);

    int H = 0, W = 0;
    {
        std::stringstream ss(inside);
        char comma = 0;
        ss >> H;
        ss >> comma;
        ss >> W;
        if (H <= 0 || W <= 0) throw std::runtime_error("NPY: invalid shape values");
    }

    NpyArrayI16 arr;
    arr.H = H;
    arr.W = W;
    arr.data.resize(static_cast<size_t>(H) * static_cast<size_t>(W));

    ifs.read(reinterpret_cast<char*>(arr.data.data()),
             static_cast<std::streamsize>(arr.data.size() * sizeof(int16_t)));
    if (!ifs) throw std::runtime_error("NPY: read array data failed");

    return arr;
}

inline GridMap gridMapFromOcc(const NpyArrayI16& occ, float resolution) {
    GridMap m;
    m.width = occ.W;
    m.height = occ.H;
    m.resolution = resolution;
    // coordinate convention: bottom-left is (0,0)
    m.origin_x = 0.0f;
    m.origin_y = 0.0f;

    m.data.resize(static_cast<size_t>(m.width) * static_cast<size_t>(m.height));
    for (int y = 0; y < m.height; ++y) {
        for (int x = 0; x < m.width; ++x) {
            int16_t v = occ.data[static_cast<size_t>(y) * m.width + x];
            int8_t out = -1;
            if (v <= -1) out = -1;
            else if (v >= 100) out = 100;
            else out = 0;
            m.data[static_cast<size_t>(y) * m.width + x] = out;
        }
    }
    return m;
}

// =========================
// Simplified LaserScan: distance to map bounding box
// (Fast, stable. Later you can upgrade to grid raycast.)
// =========================
inline float rayToRectBoundary(float x, float y, float ang, float xmin, float xmax, float ymin, float ymax) {
    float dx = std::cos(ang);
    float dy = std::sin(ang);
    const float eps = 1e-6f;
    float tmin = 1e9f;

    if (std::fabs(dx) > eps) {
        float tx1 = (xmin - x) / dx;
        float tx2 = (xmax - x) / dx;
        if (tx1 > 0) tmin = std::min(tmin, tx1);
        if (tx2 > 0) tmin = std::min(tmin, tx2);
    }
    if (std::fabs(dy) > eps) {
        float ty1 = (ymin - y) / dy;
        float ty2 = (ymax - y) / dy;
        if (ty1 > 0) tmin = std::min(tmin, ty1);
        if (ty2 > 0) tmin = std::min(tmin, ty2);
    }
    return std::clamp(tmin, 0.05f, 8.0f);
}

// =========================
// Scheduler
// =========================
struct StreamSched {
    uint32_t type;
    uint64_t period_ns;
    uint64_t next_ts;
};

inline uint64_t periodNsFromHz(uint64_t hz) {
    return static_cast<uint64_t>(1'000'000'000ull / hz);
}

// =========================
// Main API: generate log from floorplan assets
// =========================
inline void generateFakeCleaningLogFromFloorplanAssets(
    const std::string& log_path,
    const std::string& idx_path,
    const std::string& grid_npy_path,
    const std::string& traj_csv_path,
    double duration_sec
) {
    if (duration_sec <= 0.0) throw std::runtime_error("duration_sec must be > 0");

    // Frequencies
    const uint64_t hz_state = 10;
    const uint64_t hz_pose  = 20;
    const uint64_t hz_imu   = 50;
    const uint64_t hz_odom  = 50;
    const uint64_t hz_slip  = 20;
    const uint64_t hz_map   = 1;
    const uint64_t hz_scan  = 5;

    // Load assets
    auto occ = loadNpyInt16_2D(grid_npy_path);
    const float res = 0.05f;
    GridMap grid = gridMapFromOcc(occ, res);

    auto traj = loadTrajectoryCSV(traj_csv_path);

    // Fixed duration window
    uint64_t t0 = traj.front().ts;
    uint64_t t_end = t0 + static_cast<uint64_t>(duration_sec * 1e9);
    if (t_end > traj.back().ts) t_end = traj.back().ts;

    // Noise
    std::mt19937 rng(42);
    std::normal_distribution<float> n_pose(0.0f, 0.01f);
    std::normal_distribution<float> n_ang (0.0f, 0.01f);
    std::normal_distribution<float> n_imu (0.0f, 0.005f);
    std::normal_distribution<float> n_scan(0.0f, 0.02f);
    std::bernoulli_distribution slip_event(0.002);

    // Map bounds in meters
    const float xmin = grid.origin_x;
    const float ymin = grid.origin_y;
    const float xmax = grid.origin_x + grid.width  * grid.resolution;
    const float ymax = grid.origin_y + grid.height * grid.resolution;

    auto stateAt = [&](double t_sec)->RobotState {
        if (t_sec < 1.0) return RobotState::selfcheck;
        if (t_sec < 2.0) return RobotState::findwall;
        if (t_sec < duration_sec - 3.0) return RobotState::coverage;
        if (t_sec < duration_sec - 1.0) return RobotState::gohome;
        return RobotState::end;
    };

    std::vector<StreamSched> streams = {
        {TYPE_STATE, periodNsFromHz(hz_state), t0},
        {TYPE_POSE,  periodNsFromHz(hz_pose),  t0},
        {TYPE_IMU,   periodNsFromHz(hz_imu),   t0},
        {TYPE_ODOM,  periodNsFromHz(hz_odom),  t0},
        {TYPE_SLIP,  periodNsFromHz(hz_slip),  t0},
        {TYPE_MAP,   periodNsFromHz(hz_map),   t0},
        {TYPE_SCAN,  periodNsFromHz(hz_scan),  t0},
    };

    LogWriter writer(log_path);
    IndexDB db;

    PoseSample last_true = samplePoseNearest(traj, t0);

    while (true) {
        auto it = std::min_element(streams.begin(), streams.end(),
                                   [](const StreamSched& a, const StreamSched& b){ return a.next_ts < b.next_ts; });

        uint64_t ts = it->next_ts;
        if (ts >= t_end) break;

        double t_sec = (ts - t0) / 1e9;
        PoseSample true_pose = samplePoseNearest(traj, ts);

        Record rec;

        switch (it->type) {
            case TYPE_STATE: {
                rec = pushRecordData(TYPE_STATE, stateAt(t_sec), ts);
                break;
            }
            case TYPE_POSE: {
                Pose2D p{};
                p.x = true_pose.x + n_pose(rng);
                p.y = true_pose.y + n_pose(rng);
                p.angle = true_pose.a + n_ang(rng);
                rec = pushRecordData(TYPE_POSE, p, ts);
                break;
            }
            case TYPE_IMU: {
                Imu imu{};
                imu.yaw = true_pose.a + n_imu(rng);
                imu.pitch = n_imu(rng) * 0.5f;
                imu.roll  = n_imu(rng) * 0.5f;
                rec = pushRecordData(TYPE_IMU, imu, ts);
                break;
            }
            case TYPE_ODOM: {
                Odom od{};
                od.fuse_pose = Pose2D{true_pose.x, true_pose.y, true_pose.a};
                od.fuse_pose.x += n_pose(rng) * 0.5f;
                od.fuse_pose.y += n_pose(rng) * 0.5f;
                od.fuse_pose.angle += n_ang(rng) * 0.5f;

                float dx = (true_pose.x - last_true.x);
                float dy = (true_pose.y - last_true.y);
                float da = (true_pose.a - last_true.a);

                od.raw_pose.x = last_true.x + dx + n_pose(rng) * 1.2f;
                od.raw_pose.y = last_true.y + dy + n_pose(rng) * 1.2f;
                od.raw_pose.angle = last_true.a + da + n_ang(rng) * 1.2f;

                rec = pushRecordData(TYPE_ODOM, od, ts);
                break;
            }
            case TYPE_SLIP: {
                Slip sl{};
                bool e = slip_event(rng);
                sl.line_slip = e;
                sl.rotate_slip = e && (std::fabs(std::sin(true_pose.a)) > 0.5f);
                rec = pushRecordData(TYPE_SLIP, sl, ts);
                break;
            }
            case TYPE_MAP: {
                rec = pushRecordData(TYPE_MAP, grid, ts);
                break;
            }
            case TYPE_SCAN: {
                LaserScan scan{};
                scan.stamp_ns = ts;
                scan.angle_min = 0.f;
                scan.angle_increment = static_cast<float>(2.0 * 3.1415926 / 360.0);
                scan.beams.resize(360);

                for (int i = 0; i < 360; ++i) {
                    float ang = scan.angle_min + i * scan.angle_increment + true_pose.a;
                    float d = rayToRectBoundary(true_pose.x, true_pose.y, ang, xmin, xmax, ymin, ymax);
                    d += n_scan(rng);
                    scan.beams[i] = std::clamp(d, 0.05f, 8.0f);
                }

                rec = pushRecordData(TYPE_SCAN, scan, ts);
                break;
            }
            default:
                throw std::runtime_error("unknown type in generator");
        }

        uint64_t offset = writer.writeRecord(rec);
        db.all.push_back(IndexItem{rec.timestamp, offset, rec.type});

        it->next_ts += it->period_ns;
        last_true = true_pose;
    }

    std::sort(db.all.begin(), db.all.end(),
              [](const IndexItem& a, const IndexItem& b){ return a.timestamp_ns < b.timestamp_ns; });
    db.buildTypeViews();
    saveIndex(idx_path, db);
}
