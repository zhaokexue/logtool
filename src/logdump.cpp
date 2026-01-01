#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

#include "log_io.hpp"
#include "codec.hpp"
#include "log_types.hpp"

static const char* typeName(uint32_t t){
    switch (t){
        case TYPE_CLEAN_STATE: return "CLEAN_STATE";
        case TYPE_EXCEPTION_DATA: return "EXCEPTION";
        case TYPE_MOTION_STATE: return "MOTION_STATE";
        case TYPE_SENSOR_DATA: return "SENSOR_DATA";
        case TYPE_GRID_MAP: return "GRID_MAP";
        case TYPE_EXPROATION_GRID_MAP: return "EXPLORATION_GRID_MAP";
        case TYPE_ROBOT_LIDAR: return "ROBOT_LIDAR";
        case TYPE_ROBOT_POSE: return "ROBOT_POSE";
        case TYPE_ROBOT_REALTIME_POSE: return "ROBOT_REALTIME_POSE";
        case TYPE_GLOBAL_PATH_SET: return "GLOBAL_PATH_SET";
        case TYPE_LOCAL_PATH_SET: return "LOCAL_PATH_SET";
        case TYPE_DOOR_POINT_SET: return "DOOR_POINT_SET";
        case TYPE_PLAN_POINT_SET: return "PLAN_POINT_SET";
        case TYPE_COVER_POINT_SET: return "COVER_POINT_SET";
        case TYPE_CHAIN_LINE_SET: return "CHAIN_LINE_SET";
        case TYPE_PARTICLE_SET: return "PARTICLE_SET";
        case TYPE_COVERAGE_AREA_SET: return "COVERAGE_AREA_SET";
        case TYPE_POINT_DEBUG1: return "POINT_DEBUG1";
        case TYPE_POINT_DEBUG2: return "POINT_DEBUG2";
        case TYPE_POINT_DEBUG3: return "POINT_DEBUG3";
        default: return "UNKNOWN";
    }
}

int main(int argc, char** argv){
    try{
        std::string log_path = "data/fake_cleaning.log";
        size_t limit = 50;

        for (int i=1; i+1<argc; ++i){
            std::string k = argv[i];
            if (k=="--log") log_path = argv[i+1];
            if (k=="--limit") limit = static_cast<size_t>(std::stoull(argv[i+1]));
        }

        std::ifstream ifs(log_path, std::ios::binary);
        if (!ifs) throw std::runtime_error("cannot open log: " + log_path);

        FileHeader fh{};
        ifs.read(reinterpret_cast<char*>(&fh), sizeof(fh));
        if (!ifs) throw std::runtime_error("read FileHeader failed");
        if (fh.magic != 0x31474F4C) throw std::runtime_error("bad log magic");
        if (fh.version != 1) throw std::runtime_error("unsupported log version");

        std::cout << "Log: " << log_path << "\n";

        size_t nrec = 0;
        while (nrec < limit){
            RecordHeader h{};
            ifs.read(reinterpret_cast<char*>(&h), sizeof(h));
            if (!ifs) break;

            Record r;
            r.type = h.type;
            r.timestamp = h.timestamp_ns;
            r.length = h.length;
            r.data.resize(h.length);
            if (h.length){
                ifs.read(reinterpret_cast<char*>(r.data.data()), h.length);
                if (!ifs) throw std::runtime_error("read payload failed");
            }

            std::cout << "--------------------------------------------------\n";
            std::cout << "#" << nrec
                      << " type=" << r.type << " (" << typeName(r.type) << ")"
                      << " ts=" << r.timestamp
                      << " len=" << r.length << "\n";

            // Light decode for common types
            if (r.type == TYPE_ROBOT_REALTIME_POSE || r.type == TYPE_ROBOT_POSE){
                Pose2D p = parseRecordPayload<Pose2D>(r);
                std::cout << "Pose2D: x=" << p.x << " y=" << p.y << " yaw=" << p.angle << "\n";
            } else if (r.type == TYPE_ROBOT_LIDAR){
                LaserScan s = parseRecordPayload<LaserScan>(r);
                std::cout << "LaserScan: beams=" << s.beams.size() << " angle_min=" << s.angle_min
                          << " angle_inc=" << s.angle_increment << "\n";
            } else if (r.type == TYPE_GRID_MAP || r.type == TYPE_EXPROATION_GRID_MAP){
                GridMap m = parseRecordPayload<GridMap>(r);
                std::cout << "GridMap: " << m.width << "x" << m.height << " res=" << m.resolution
                          << " origin=(" << m.origin_x << "," << m.origin_y << ")" << "\n";
            } else if (r.type == TYPE_SENSOR_DATA){
                SensorData sd = parseRecordPayload<SensorData>(r);
                std::cout << "SensorData: ctrl_v=" << sd.ctrl_v << " ctrl_w=" << sd.ctrl_w
                          << " batt=" << sd.battery_voltage << "\n";
            }

            ++nrec;
        }
        return 0;

    } catch (const std::exception& e){
        std::cerr << "dump_log failed: " << e.what() << "\n";
        return 1;
    }
}
