#include <iostream>
#include <unordered_map>
#include <fstream>
#include <string>
#include <filesystem>
#include <cstdlib>

#include "log_types.hpp"
#include "log_io.hpp"
#include "index.hpp"
#include "stats.hpp"
#include "codec.hpp"
#include "fake_data.hpp"

static void scanStats(const std::string& log_path,
                      std::unordered_map<uint32_t, TypeStats>& stats) {
    std::ifstream ifs(log_path, std::ios::binary);
    if (!ifs) throw std::runtime_error("cannot open log for stats");

    FileHeader fh{};
    ifs.read(reinterpret_cast<char*>(&fh), sizeof(fh));
    if (!ifs || fh.magic != 0x31474F4C) throw std::runtime_error("bad log header");

    while (true) {
        RecordHeader h{};
        ifs.read(reinterpret_cast<char*>(&h), sizeof(h));
        if (!ifs) break;

        updateStats(stats, h.type, h.timestamp_ns, h.length);

        ifs.seekg(static_cast<std::streamoff>(h.length), std::ios::cur);
        if (!ifs) throw std::runtime_error("seek payload failed");
    }
}

static void demoDragPreview(const std::string& log_path,
                            const IndexDB& db,
                            uint64_t target_ts) {
    LogReader reader(log_path);

    auto pick = [&](uint32_t type) {
        const IndexItem* it = db.nearest(type, target_ts);
        if (!it) {
            std::cout << "type " << type << " not found\n";
            return;
        }
        Record r = reader.readAt(it->offset);
        std::cout << "type " << type
                  << " nearest_ts=" << r.timestamp
                  << " offset=" << it->offset
                  << " len=" << r.length << "\n";

        if (type == TYPE_ROBOT_REALTIME_POSE || type == TYPE_ROBOT_POSE) {
            auto p = parseRecordPayload<Pose2D>(r);
            std::cout << "  pose=(" << p.x << "," << p.y << ") a=" << p.angle << "\n";
        } else if (type == TYPE_GRID_MAP || type == TYPE_EXPROATION_GRID_MAP) {
            auto m = parseRecordPayload<GridMap>(r);
            std::cout << "  map=" << m.width << "x" << m.height << " res=" << m.resolution << "\n";
        } else if (type == TYPE_ROBOT_LIDAR) {
            auto s = parseRecordPayload<LaserScan>(r);
            std::cout << "  scan beams=" << s.beams.size() << " a0=" << s.angle_min
                      << " da=" << s.angle_increment << "\n";
        } else if (type == TYPE_SENSOR_DATA) {
            auto sd = parseRecordPayload<SensorData>(r);
            std::cout << "  sensor ctrl_v=" << sd.ctrl_v << " ctrl_w=" << sd.ctrl_w
                      << " batt=" << sd.battery_voltage << "\n";
        }
    };

    pick(TYPE_CLEAN_STATE);
    pick(TYPE_EXCEPTION_DATA);
    pick(TYPE_MOTION_STATE);
    pick(TYPE_SENSOR_DATA);
    pick(TYPE_GRID_MAP);
    pick(TYPE_ROBOT_REALTIME_POSE);
    pick(TYPE_ROBOT_POSE);
    pick(TYPE_ROBOT_LIDAR);
}

static std::string argValue(int argc, char** argv, const std::string& key, const std::string& def="") {
    for (int i = 1; i + 1 < argc; ++i) {
        if (argv[i] == key) return argv[i + 1];
    }
    return def;
}

static bool hasFlag(int argc, char** argv, const std::string& key) {
    for (int i = 1; i < argc; ++i) if (argv[i] == key) return true;
    return false;
}

int main(int argc, char** argv) {
    try {
        std::string out_log = argValue(argc, argv, "--out", "fake_cleaning.log");
        std::string idx_path = out_log + ".idx";
        double duration_sec = 120.0;
        {
            std::string d = argValue(argc, argv, "--duration", "");
            if (!d.empty()) duration_sec = std::stod(d);
        }

        if (!hasFlag(argc, argv, "--from_floorplan")) {
            throw std::runtime_error(
                "Usage:\n"
                "  ./logtool --from_floorplan --img <path> [--duration 120] [--out fake_cleaning.log]\n"
            );
        }

        std::string img_path = argValue(argc, argv, "--img", "");
        if (img_path.empty()) throw std::runtime_error("--from_floorplan requires --img <path>");

        std::filesystem::create_directories("assets");
        std::string grid_npy = "assets/grid_map.npy";
        std::string traj_csv = "assets/trajectory.csv";

        // Call python preprocessor
        std::string cmd =
            "python3 tools/floorplan_to_map_and_traj.py"
            " --img \"" + img_path + "\""
            " --out_map \"" + grid_npy + "\""
            " --out_traj \"" + traj_csv + "\""
            " --height_m 10 --res 0.05 --lane_step_m 0.30 --pose_hz 20 --speed 0.25";

        std::cout << "Running: " << cmd << "\n";
        int rc = std::system(cmd.c_str());
        if (rc != 0) throw std::runtime_error("python preprocessing failed, rc=" + std::to_string(rc));

        std::cout << "Generating log from assets:\n"
                  << "  grid=" << grid_npy << "\n"
                  << "  traj=" << traj_csv << "\n"
                  << "  out=" << out_log << "\n"
                  << "  duration=" << duration_sec << "s\n";

        generateFakeCleaningLogFromFloorplanAssets(out_log, idx_path, grid_npy, traj_csv, duration_sec);

        // Stats + preview
        std::unordered_map<uint32_t, TypeStats> stats;
        scanStats(out_log, stats);
        printStats(stats);

        IndexDB db = loadIndex(idx_path);

        // Drag preview at mid time
        uint64_t t0 = db.all.empty() ? 0 : db.all.front().timestamp_ns;
        uint64_t target_ts = t0 + static_cast<uint64_t>((duration_sec * 0.5) * 1e9);

        std::cout << "\nDrag preview target_ts=" << target_ts << "\n";
        demoDragPreview(out_log, db, target_ts);

        std::cout << "\nDone.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        return 1;
    }
}
