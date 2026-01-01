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
#include <limits>

#include "log_types.hpp"
#include "codec.hpp"
#include "log_io.hpp"
#include "index.hpp"

// =========================================
// Floorplan-assets -> fake cleaning log generator (vNext protocol)
// Inputs:
//  - assets/grid_map.npy : int16 2D grid (-1 unknown, 0 free, 100 occupied)
//  - assets/trajectory.csv : columns timestamp_ns,x_m,y_m,angle_rad
// Output:
//  - binary log + index
// =========================================

// -------------------------
// CSV trajectory loader
// -------------------------
struct PoseSample {
    uint64_t ts{0};
    float x{0}, y{0}, a{0};
};

inline std::vector<PoseSample> loadTrajectoryCSV(const std::string& csv_path) {
    std::ifstream ifs(csv_path);
    if (!ifs) throw std::runtime_error("loadTrajectoryCSV: cannot open " + csv_path);

    std::string line;
    std::getline(ifs, line); // header

    std::vector<PoseSample> out;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string tok;
        PoseSample p;
        std::getline(ss, tok, ','); p.ts = static_cast<uint64_t>(std::stoull(tok));
        std::getline(ss, tok, ','); p.x  = static_cast<float>(std::stof(tok));
        std::getline(ss, tok, ','); p.y  = static_cast<float>(std::stof(tok));
        std::getline(ss, tok, ','); p.a  = static_cast<float>(std::stof(tok));
        out.push_back(p);
    }
    if (out.size() < 2) throw std::runtime_error("loadTrajectoryCSV: trajectory too short");
    std::sort(out.begin(), out.end(), [](const PoseSample& a, const PoseSample& b){ return a.ts < b.ts; });
    return out;
}

// -------------------------
// Minimal .npy (v1.0) loader for int16 2D
// -------------------------
struct NpyArrayI16 {
    int rows{0};
    int cols{0};
    std::vector<int16_t> data; // row-major
};

inline NpyArrayI16 loadNpyInt16_2D(const std::string& npy_path) {
    std::ifstream ifs(npy_path, std::ios::binary);
    if (!ifs) throw std::runtime_error("loadNpyInt16_2D: cannot open " + npy_path);

    // magic
    char magic[6] = {0};
    ifs.read(magic, 6);
    if (std::strncmp(magic, "\x93NUMPY", 6) != 0) throw std::runtime_error("bad npy magic");

    uint8_t vmaj=0, vmin=0;
    ifs.read(reinterpret_cast<char*>(&vmaj), 1);
    ifs.read(reinterpret_cast<char*>(&vmin), 1);

    uint16_t header_len = 0;
    ifs.read(reinterpret_cast<char*>(&header_len), 2);

    std::string header(header_len, '\0');
    ifs.read(header.data(), header_len);

    // very small header parser (assumes little-endian, C-order)
    // Expect: {'descr': '<i2', 'fortran_order': False, 'shape': (H, W), }
    if (header.find("'<i2'") == std::string::npos && header.find("\"<i2\"") == std::string::npos) {
        throw std::runtime_error("npy dtype is not <i2");
    }
    if (header.find("fortran_order") != std::string::npos && header.find("True") != std::string::npos) {
        throw std::runtime_error("npy fortran_order not supported");
    }
    auto shp = header.find("shape");
    if (shp == std::string::npos) throw std::runtime_error("npy header missing shape");
    auto lp = header.find('(', shp);
    auto rp = header.find(')', shp);
    if (lp == std::string::npos || rp == std::string::npos || rp <= lp) throw std::runtime_error("bad shape");
    std::string inside = header.substr(lp + 1, rp - lp - 1);
    inside.erase(std::remove_if(inside.begin(), inside.end(), [](char c){ return c==' ' || c=='\n' || c=='\t'; }), inside.end());

    int H=0, W=0;
    {
        auto comma = inside.find(',');
        if (comma == std::string::npos) throw std::runtime_error("bad shape tuple");
        H = std::stoi(inside.substr(0, comma));
        W = std::stoi(inside.substr(comma + 1));
    }
    if (H <= 0 || W <= 0) throw std::runtime_error("bad npy shape");

    NpyArrayI16 a;
    a.rows = H;
    a.cols = W;
    a.data.resize(static_cast<size_t>(H) * static_cast<size_t>(W));

    ifs.read(reinterpret_cast<char*>(a.data.data()), static_cast<std::streamsize>(a.data.size() * sizeof(int16_t)));
    if (!ifs) throw std::runtime_error("npy data read failed");
    return a;
}

// -------------------------
// Path set builders (global/local) from reference trajectory
// -------------------------
// We emit Pose2DSet records for global/local paths at low frequency.
// The UI only needs x/y; angle is kept for completeness.
inline Pose2DSet buildGlobalPathFromTraj(const std::vector<PoseSample>& traj,
                                        size_t start_idx,
                                        float min_step_m = 0.20f,
                                        size_t max_points = 600) {
    Pose2DSet out;
    if (traj.empty()) return out;
    start_idx = std::min(start_idx, traj.size() - 1);

    float lastx = traj[start_idx].x;
    float lasty = traj[start_idx].y;
    out.pose_set.push_back(Pose2D{lastx, lasty, traj[start_idx].a});

    const float min_step2 = min_step_m * min_step_m;
    for (size_t i = start_idx + 1; i < traj.size() && out.pose_set.size() < max_points; ++i) {
        const float dx = traj[i].x - lastx;
        const float dy = traj[i].y - lasty;
        if ((dx * dx + dy * dy) < min_step2) continue;
        out.pose_set.push_back(Pose2D{traj[i].x, traj[i].y, traj[i].a});
        lastx = traj[i].x;
        lasty = traj[i].y;
    }
    return out;
}

inline Pose2DSet buildLocalPathFromTraj(const std::vector<PoseSample>& traj,
                                       size_t start_idx,
                                       float horizon_m = 3.0f,
                                       float min_step_m = 0.10f,
                                       size_t max_points = 200) {
    Pose2DSet out;
    if (traj.empty()) return out;
    start_idx = std::min(start_idx, traj.size() - 1);

    float lastx = traj[start_idx].x;
    float lasty = traj[start_idx].y;
    out.pose_set.push_back(Pose2D{lastx, lasty, traj[start_idx].a});

    const float horizon2 = horizon_m * horizon_m;
    const float min_step2 = min_step_m * min_step_m;
    for (size_t i = start_idx + 1; i < traj.size() && out.pose_set.size() < max_points; ++i) {
        // stop if we reached horizon distance from the start point
        const float dhx = traj[i].x - traj[start_idx].x;
        const float dhy = traj[i].y - traj[start_idx].y;
        if ((dhx * dhx + dhy * dhy) > horizon2) break;

        const float dx = traj[i].x - lastx;
        const float dy = traj[i].y - lasty;
        if ((dx * dx + dy * dy) < min_step2) continue;
        out.pose_set.push_back(Pose2D{traj[i].x, traj[i].y, traj[i].a});
        lastx = traj[i].x;
        lasty = traj[i].y;
    }
    return out;
}

// -------------------------
// Geometry helpers
// -------------------------
inline void worldToCell(float wx, float wy, float origin_x, float origin_y, float res, int& cx, int& cy) {
    cx = static_cast<int>(std::floor((wx - origin_x) / res));
    cy = static_cast<int>(std::floor((wy - origin_y) / res));
}

inline void worldToLocal(float wx, float wy, float px, float py, float yaw, float& lx, float& ly) {
    // local: x forward, y left
    const float dx = wx - px;
    const float dy = wy - py;
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    lx =  c * dx + s * dy;
    ly = -s * dx + c * dy;
}

inline void localToWorld(float lx, float ly, float px, float py, float yaw, float& wx, float& wy) {
    // local: x forward, y left
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    wx = px + lx * c - ly * s;
    wy = py + lx * s + ly * c;
}

// DDA raycast on occupancy grid (occ: int16, -1 unknown, 0 free, 100 occupied)
inline float raycastGridHit(const NpyArrayI16& occ,
                            float px, float py, float ang_world,
                            float origin_x, float origin_y, float res,
                            float max_range_m,
                            bool hit_unknown,
                            float& hx, float& hy) {
    const int H = occ.rows;
    const int W = occ.cols;
    const float dx = std::cos(ang_world);
    const float dy = std::sin(ang_world);
    const float eps = 1e-9f;

    int cx=0, cy=0;
    worldToCell(px, py, origin_x, origin_y, res, cx, cy);
    if (cx < 0 || cx >= W || cy < 0 || cy >= H) {
        hx = px + max_range_m * dx;
        hy = py + max_range_m * dy;
        return max_range_m;
    }

    auto at = [&](int x, int y)->int16_t { return occ.data[static_cast<size_t>(y) * static_cast<size_t>(W) + static_cast<size_t>(x)]; };

    // check starting cell
    {
        const int16_t v0 = at(cx, cy);
        if ((hit_unknown && v0 == -1) || (v0 >= 100)) {
            hx = px; hy = py;
            return 0.f;
        }
    }

    const int step_x = (dx > 0) ? 1 : -1;
    const int step_y = (dy > 0) ? 1 : -1;

    float tMaxX = std::numeric_limits<float>::infinity();
    float tMaxY = std::numeric_limits<float>::infinity();
    float tDeltaX = std::numeric_limits<float>::infinity();
    float tDeltaY = std::numeric_limits<float>::infinity();

    if (std::abs(dx) >= eps) {
        const float next_vert = origin_x + (cx + (dx > 0 ? 1 : 0)) * res;
        tMaxX = (next_vert - px) / dx;
        tDeltaX = res / std::abs(dx);
    }
    if (std::abs(dy) >= eps) {
        const float next_horz = origin_y + (cy + (dy > 0 ? 1 : 0)) * res;
        tMaxY = (next_horz - py) / dy;
        tDeltaY = res / std::abs(dy);
    }

    float t = 0.f;
    while (t <= max_range_m) {
        if (tMaxX < tMaxY) {
            cx += step_x;
            t = tMaxX;
            tMaxX += tDeltaX;
        } else {
            cy += step_y;
            t = tMaxY;
            tMaxY += tDeltaY;
        }

        if (t > max_range_m) break;

        if (cx < 0 || cx >= W || cy < 0 || cy >= H) {
            hx = px + t * dx;
            hy = py + t * dy;
            return t;
        }

        const int16_t v = at(cx, cy);
        if (hit_unknown && v == -1) {
            hx = px + t * dx;
            hy = py + t * dy;
            return t;
        }
        if (v >= 100) {
            hx = px + t * dx;
            hy = py + t * dy;
            return t;
        }
    }

    hx = px + max_range_m * dx;
    hy = py + max_range_m * dy;
    return max_range_m;
}

// -------------------------
// Sampling helpers
// -------------------------
inline PoseSample samplePoseAt(const std::vector<PoseSample>& traj, uint64_t ts_ns) {
    // hold-last (nearest <= ts)
    auto it = std::upper_bound(traj.begin(), traj.end(), ts_ns,
                               [](uint64_t t, const PoseSample& p){ return t < p.ts; });
    if (it == traj.begin()) return traj.front();
    return *(it - 1);
}

inline CleanState cleanStateAt(double progress01) {
    // progress partition similar to previous generator
    if (progress01 < 0.05) return CleanState::selfcheck;
    if (progress01 < 0.10) return CleanState::checkdock;
    if (progress01 < 0.20) return CleanState::findwall;
    if (progress01 < 0.35) return CleanState::followwall;
    if (progress01 < 0.90) return CleanState::coverage;
    if (progress01 < 0.98) return CleanState::gohome;
    return CleanState::end;
}

// -------------------------
// Main generator
// -------------------------
inline void generateFakeCleaningLogFromFloorplanAssets(
    const std::string& out_log_path,
    const std::string& out_idx_path,
    const std::string& grid_npy_path,
    const std::string& traj_csv_path,
    double duration_sec) {

    // Load assets
    const auto occ = loadNpyInt16_2D(grid_npy_path);
    const auto traj = loadTrajectoryCSV(traj_csv_path);

    const float res_m = 0.05f;          // assets generator uses 0.05 by default
    const float origin_x = 0.f;
    const float origin_y = 0.f;

    const uint64_t t0 = traj.front().ts;
    const uint64_t t1 = traj.back().ts;
    const double asset_dur = (t1 > t0) ? double(t1 - t0) / 1e9 : duration_sec;
    const double dur = (duration_sec > 0) ? std::min(duration_sec, asset_dur) : asset_dur;
    const uint64_t tend = t0 + static_cast<uint64_t>(dur * 1e9);

    LogWriter writer(out_log_path);
    IndexDB idx;

    // Deterministic RNG
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> uni01(0.f, 1.f);
    std::normal_distribution<float> n01(0.f, 1.f);

    // Pre-build constant map (1px per cell)
    GridMap gmap;
    gmap.width = occ.cols;
    gmap.height = occ.rows;
    gmap.resolution = res_m;
    gmap.origin_x = origin_x;
    gmap.origin_y = origin_y;
    gmap.data.resize(static_cast<size_t>(gmap.width) * static_cast<size_t>(gmap.height));

    // Convert int16(-1/0/100) -> int8(-1/0/100)
    for (int y = 0; y < gmap.height; ++y) {
        for (int x = 0; x < gmap.width; ++x) {
            const int16_t v = occ.data[static_cast<size_t>(y) * static_cast<size_t>(occ.cols) + static_cast<size_t>(x)];
            int8_t o = 0;
            if (v < 0) o = -1;
            else if (v >= 100) o = 100;
            else o = 0;
            gmap.data[static_cast<size_t>(y) * static_cast<size_t>(gmap.width) + static_cast<size_t>(x)] = o;
        }
    }

    auto write = [&](uint32_t type, uint64_t ts, const auto& payload) {
        Record r;
        r.type = type;
        r.timestamp = ts;
        r.data = serializePayload(payload);
        r.length = static_cast<uint32_t>(r.data.size());

        const uint64_t off = writer.writeRecord(r);
        idx.all.push_back(IndexItem{r.timestamp, off, r.type});
    };

    // Frequencies (per protocol comments)
    const double hz_clean = 5.0;
    const double hz_exc   = 5.0;
    const double hz_motion= 5.0;
    const double hz_sensor= 50.0;
    const double hz_map   = 1.0;
    const double hz_path  = 2.0;   // global/local path updates (low rate)
    const double hz_pose_rt = 20.0;
    const double hz_pose_lidar = 6.0;
    const double hz_lidar = 6.0;

    uint64_t next_clean  = t0;
    uint64_t next_exc    = t0;
    uint64_t next_motion = t0;
    uint64_t next_sensor = t0;
    uint64_t next_map    = t0;
    uint64_t next_gpath  = t0;
    uint64_t next_lpath  = t0;
    uint64_t next_pose_rt = t0;
    uint64_t next_pose_lidar = t0;
    uint64_t next_lidar  = t0;

    const uint64_t step_clean  = static_cast<uint64_t>(1e9 / hz_clean);
    const uint64_t step_exc    = static_cast<uint64_t>(1e9 / hz_exc);
    const uint64_t step_motion = static_cast<uint64_t>(1e9 / hz_motion);
    const uint64_t step_sensor = static_cast<uint64_t>(1e9 / hz_sensor);
    const uint64_t step_map    = static_cast<uint64_t>(1e9 / hz_map);
    const uint64_t step_path   = static_cast<uint64_t>(1e9 / hz_path);
    const uint64_t step_pose_rt= static_cast<uint64_t>(1e9 / hz_pose_rt);
    const uint64_t step_pose_lidar= static_cast<uint64_t>(1e9 / hz_pose_lidar);
    const uint64_t step_lidar  = static_cast<uint64_t>(1e9 / hz_lidar);

    // Default states
    CleanState clean = CleanState::selfcheck;
    ExceptionCode exc = ExceptionCode::low_power;
    MotionState motion = MotionState::line;

    // Keep last commanded velocities for motion state synthesis
    float last_ctrl_v = 0.f;
    float last_ctrl_w = 0.f;

    // Generate timeline
    for (uint64_t ts = t0; ts <= tend; ts += 1000000ULL) { // 1ms timebase
        const double prog = (tend > t0) ? (double(ts - t0) / double(tend - t0)) : 1.0;

        // Map (1Hz)
        if (ts >= next_map) {
            write(TYPE_GRID_MAP, ts, gmap);
            next_map += step_map;
        }

        // ----- Simulated planner paths (global/local) -----
        // Keep payload size bounded: downsample by distance + cap points.
        if (ts >= next_gpath || ts >= next_lpath) {
            const double tsec = double(ts - t0) / 1e9;
            size_t idx = static_cast<size_t>(std::llround(tsec * hz_pose_rt));
            if (idx >= traj.size()) idx = traj.empty() ? 0 : (traj.size() - 1);

            if (ts >= next_gpath) {
                Pose2DSet gset = buildGlobalPathFromTraj(traj, idx);
                write(TYPE_GLOBAL_PATH_SET, ts, gset);
                next_gpath += step_path;
            }
            if (ts >= next_lpath) {
                Pose2DSet lset = buildLocalPathFromTraj(traj, idx);
                write(TYPE_LOCAL_PATH_SET, ts, lset);
                next_lpath += step_path;
            }
        }

        // Robot realtime pose (20Hz)
        if (ts >= next_pose_rt) {
            const PoseSample p = samplePoseAt(traj, ts);
            Pose2D pose{p.x, p.y, p.a};
            write(TYPE_ROBOT_REALTIME_POSE, ts, pose);
            next_pose_rt += step_pose_rt;
        }

        // Lidar pose (6Hz)
        if (ts >= next_pose_lidar) {
            const PoseSample p = samplePoseAt(traj, ts);
            Pose2D pose{p.x, p.y, p.a};
            write(TYPE_ROBOT_POSE, ts, pose);
            next_pose_lidar += step_pose_lidar;
        }

        // Lidar scan (6Hz)
        if (ts >= next_lidar) {
            const PoseSample p = samplePoseAt(traj, ts);
            LaserScan scan;
            scan.stamp_ns = ts;
            scan.angle_min = 0.0f;
            scan.angle_increment = static_cast<float>(2.0 * M_PI / 360.0);
            scan.beams.resize(360);

            for (size_t i = 0; i < scan.beams.size(); ++i) {
                const float ang_world = p.a + scan.angle_min + static_cast<float>(i) * scan.angle_increment;
                float hx=0.f, hy=0.f;
                const float rr = raycastGridHit(occ, p.x, p.y, ang_world, origin_x, origin_y, res_m,
                                                8.0f, true, hx, hy);
                scan.beams[i] = rr;
            }
            write(TYPE_ROBOT_LIDAR, ts, scan);
            next_lidar += step_lidar;
        }

        // SensorData (50Hz)
        if (ts >= next_sensor) {
            const PoseSample p = samplePoseAt(traj, ts);

            SensorData s;

            // commanded velocity (roughly derive from pose delta)
            // Use a tiny pseudo derivative over 50ms window.
            const uint64_t back_ts = (ts > t0 + 50000000ULL) ? (ts - 50000000ULL) : t0;
            const PoseSample pb = samplePoseAt(traj, back_ts);
            const float dt = static_cast<float>((ts - back_ts) / 1e9);
            const float vx = (dt > 1e-6f) ? (p.x - pb.x) / dt : 0.f;
            const float vy = (dt > 1e-6f) ? (p.y - pb.y) / dt : 0.f;
            const float v = std::sqrt(vx*vx + vy*vy);
            float w = (dt > 1e-6f) ? (p.a - pb.a) / dt : 0.f;

            // clamp for UI readability
            last_ctrl_v = std::min(0.6f, std::max(0.f, v));
            last_ctrl_w = std::max(-1.0f, std::min(1.0f, w));
            s.ctrl_v = last_ctrl_v;
            s.ctrl_w = last_ctrl_w;

            // slips (rare)
            const float slip_p = 0.002f;
            s.line_slip_fwd  = (uni01(rng) < slip_p);
            s.line_slip_back = (uni01(rng) < slip_p);
            s.rotate_slip_cw  = (uni01(rng) < slip_p);
            s.rotate_slip_ccw = (uni01(rng) < slip_p);

            // odom: raw has drift, fuse close to pose
            s.odom.raw_pose.x = p.x + 0.02f * n01(rng);
            s.odom.raw_pose.y = p.y + 0.02f * n01(rng);
            s.odom.raw_pose.angle = p.a + 0.02f * n01(rng);

            s.odom.fuse_pose.x = p.x + 0.005f * n01(rng);
            s.odom.fuse_pose.y = p.y + 0.005f * n01(rng);
            s.odom.fuse_pose.angle = p.a + 0.005f * n01(rng);

            // imu: small pitch/roll, yaw follows pose
            s.imu.pitch = 0.02f * n01(rng);
            s.imu.roll  = 0.02f * n01(rng);
            s.imu.yaw   = p.a;
            s.imu.imu_acc_x = 0.2f * n01(rng);
            s.imu.imu_acc_y = 0.2f * n01(rng);
            s.imu.imu_acc_z = 9.81f + 0.2f * n01(rng);
            s.imu.imu_yaw_vel = last_ctrl_w;

            // bumper/wheel-up (very rare)
            const float bump_p = 0.0008f;
            s.left_bumper  = (uni01(rng) < bump_p);
            s.right_bumper = (uni01(rng) < bump_p);
            s.left_wheel_up  = (uni01(rng) < 0.0003f);
            s.right_wheel_up = (uni01(rng) < 0.0003f);

            // right IR (wall follow): random-ish at mid phase
            s.right_ir = (prog > 0.15 && prog < 0.40) ? (uni01(rng) < 0.5f) : (uni01(rng) < 0.05f);
            s.sonar = 0.20f + 1.0f * uni01(rng);

            // cliff IR (very rare)
            const float cliff_p = 0.0002f;
            s.cliff_lr = (uni01(rng) < cliff_p);
            s.cliff_lf = (uni01(rng) < cliff_p);
            s.cliff_rf = (uni01(rng) < cliff_p);
            s.cliff_rr = (uni01(rng) < cliff_p);

            // docking IR: becomes likely near end
            const float dock_p = (prog > 0.92) ? 0.35f : 0.01f;
            s.dock_ir1 = (uni01(rng) < dock_p);
            s.dock_ir2 = (uni01(rng) < dock_p);
            s.dock_ir3 = (uni01(rng) < dock_p);
            s.dock_ir4 = (uni01(rng) < dock_p);
            s.dock_clip_state = (prog > 0.95) ? (uni01(rng) < 0.6f) : false;

            // battery voltage slowly decreases
            s.battery_voltage = 16.0f - static_cast<float>(2.0 * prog);

            write(TYPE_SENSOR_DATA, ts, s);
            next_sensor += step_sensor;
        }

        // Clean state (5Hz)
        if (ts >= next_clean) {
            clean = cleanStateAt(prog);
            write(TYPE_CLEAN_STATE, ts, clean);
            next_clean += step_clean;
        }

        // Exception (5Hz) - default none; inject a short transient at ~70%
        if (ts >= next_exc) {
            if (prog > 0.70 && prog < 0.72) exc = ExceptionCode::nai_fail;
            else if (prog > 0.90 && prog < 0.905) exc = ExceptionCode::go_home_fail;
            else exc = ExceptionCode::low_power;
            write(TYPE_EXCEPTION_DATA, ts, exc);
            next_exc += step_exc;
        }

        // Motion state (5Hz)
        if (ts >= next_motion) {
            motion = (std::abs(last_ctrl_w) > 0.5f) ? MotionState::rotate : MotionState::line;
            if (prog > 0.92) motion = MotionState::dock;
            write(TYPE_MOTION_STATE, ts, motion);
            next_motion += step_motion;
        }
    }

    std::sort(idx.all.begin(), idx.all.end(), [](const IndexItem& a, const IndexItem& b){ return a.timestamp_ns < b.timestamp_ns; });
    idx.buildTypeViews();
    saveIndex(out_idx_path, idx);
}
