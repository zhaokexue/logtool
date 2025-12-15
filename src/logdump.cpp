#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <cctype>

#include "log_io.hpp"
#include "log_types.hpp"
#include "codec.hpp"

static std::string argValue(int argc, char** argv, const std::string& key, const std::string& def="") {
    for (int i = 1; i + 1 < argc; ++i) if (argv[i] == key) return argv[i + 1];
    return def;
}
static bool hasFlag(int argc, char** argv, const std::string& key) {
    for (int i = 1; i < argc; ++i) if (argv[i] == key) return true;
    return false;
}
static uint64_t toU64(const std::string& s, uint64_t def=0) {
    if (s.empty()) return def;
    return static_cast<uint64_t>(std::stoull(s));
}
static uint32_t toU32(const std::string& s, uint32_t def=0) {
    if (s.empty()) return def;
    return static_cast<uint32_t>(std::stoul(s));
}

static void hexdumpPrefix(const std::vector<uint8_t>& data, size_t max_bytes) {
    size_t n = std::min(max_bytes, data.size());
    for (size_t i = 0; i < n; i += 16) {
        std::cout << "  " << std::setw(6) << std::setfill('0') << std::hex << i << " : ";
        for (size_t j = 0; j < 16; ++j) {
            if (i + j < n) std::cout << std::setw(2) << (int)data[i + j] << " ";
            else std::cout << "   ";
        }
        std::cout << " |";
        for (size_t j = 0; j < 16 && i + j < n; ++j) {
            uint8_t c = data[i + j];
            std::cout << (std::isprint(c) ? (char)c : '.');
        }
        std::cout << "|\n";
    }
    std::cout << std::dec << std::setfill(' ');
}

static void printParsedSummary(const Record& r) {
    try {
        if (r.type == TYPE_POSE) {
            auto p = parseRecordPayload<Pose2D>(r);
            std::cout << "  pose: x=" << p.x << " y=" << p.y << " a=" << p.angle << "\n";
        } else if (r.type == TYPE_IMU) {
            auto v = parseRecordPayload<Imu>(r);
            std::cout << "  imu: pitch=" << v.pitch << " roll=" << v.roll << " yaw=" << v.yaw << "\n";
        } else if (r.type == TYPE_ODOM) {
            auto v = parseRecordPayload<Odom>(r);
            std::cout << "  odom.raw: (" << v.raw_pose.x << "," << v.raw_pose.y << ") a=" << v.raw_pose.angle << "\n";
            std::cout << "  odom.fuse:(" << v.fuse_pose.x << "," << v.fuse_pose.y << ") a=" << v.fuse_pose.angle << "\n";
        } else if (r.type == TYPE_MAP) {
            auto m = parseRecordPayload<GridMap>(r);
            std::cout << "  map: " << m.width << "x" << m.height
                      << " res=" << m.resolution
                      << " origin=(" << m.origin_x << "," << m.origin_y << ")"
                      << " data_bytes=" << m.data.size() << "\n";
        } else if (r.type == TYPE_SCAN) {
            auto s = parseRecordPayload<LaserScan>(r);
            std::cout << "  scan: stamp_ns=" << s.stamp_ns
                      << " angle_min=" << s.angle_min
                      << " d_angle=" << s.angle_increment
                      << " beams=" << s.beams.size() << "\n";
        } else if (r.type == TYPE_SLIP) {
            auto v = parseRecordPayload<Slip>(r);
            std::cout << "  slip: line=" << (v.line_slip ? 1 : 0)
                      << " rotate=" << (v.rotate_slip ? 1 : 0) << "\n";
        } else if (r.type == TYPE_STATE) {
            auto v = parseRecordPayload<RobotState>(r);
            std::cout << "  state: " << static_cast<uint32_t>(v) << "\n";
        } else {
            std::cout << "  (no parser for type=" << r.type << ")\n";
        }
    } catch (const std::exception& e) {
        std::cout << "  parse_failed: " << e.what() << "\n";
    }
}

int main(int argc, char** argv) {
    try {
        std::string log_path = argValue(argc, argv, "--log", "");
        if (log_path.empty()) {
            std::cerr << "Usage:\n"
                      << "  ./logdump --log <file.log> [--limit N]\n"
                      << "           [--type T] [--start_ts NS] [--end_ts NS]\n"
                      << "           [--hex K] [--parse]\n";
            return 2;
        }

        uint64_t limit    = toU64(argValue(argc, argv, "--limit", "0"), 0);
        uint32_t type_f   = toU32(argValue(argc, argv, "--type", ""), 0);
        bool use_type_f   = !argValue(argc, argv, "--type", "").empty();

        uint64_t start_ts = toU64(argValue(argc, argv, "--start_ts", ""), 0);
        uint64_t end_ts   = toU64(argValue(argc, argv, "--end_ts", ""), 0);
        bool use_ts_f     = !argValue(argc, argv, "--start_ts", "").empty() || !argValue(argc, argv, "--end_ts", "").empty();

        size_t hex_k       = static_cast<size_t>(toU64(argValue(argc, argv, "--hex", "0"), 0));
        bool do_parse      = hasFlag(argc, argv, "--parse");

        uint64_t fsz = static_cast<uint64_t>(std::filesystem::file_size(log_path));

        // 低层 ifstream 扫描：我们需要 offset（RecordHeader 起点）
        std::ifstream ifs(log_path, std::ios::binary);
        if (!ifs) throw std::runtime_error("cannot open log");

        FileHeader fh{};
        ifs.read(reinterpret_cast<char*>(&fh), sizeof(fh));
        if (!ifs) throw std::runtime_error("read FileHeader failed");
        if (fh.magic != 0x31474F4C || fh.version != 1) throw std::runtime_error("bad log header"); // 与 LogReader 一致 :contentReference[oaicite:3]{index=3}

        uint64_t offset = sizeof(FileHeader);
        uint64_t shown = 0;
        uint64_t idx = 0;

        while (true) {
            if (offset + sizeof(RecordHeader) > fsz) break;

            RecordHeader h{};
            ifs.read(reinterpret_cast<char*>(&h), sizeof(h));
            if (!ifs) break;

            uint64_t payload_off = offset + sizeof(RecordHeader);
            uint64_t end_off = payload_off + h.length;

            // 一致性检查：长度越界直接报错停止
            if (end_off > fsz) {
                std::cerr << "Corrupt? idx=" << idx
                          << " offset=" << offset
                          << " type=" << h.type
                          << " ts=" << h.timestamp_ns
                          << " len=" << h.length
                          << " => end_off=" << end_off
                          << " > file_size=" << fsz << "\n";
                return 3;
            }

            bool pass = true;
            if (use_type_f && h.type != type_f) pass = false;
            if (use_ts_f) {
                if (!argValue(argc, argv, "--start_ts", "").empty() && h.timestamp_ns < start_ts) pass = false;
                if (!argValue(argc, argv, "--end_ts", "").empty()   && h.timestamp_ns > end_ts)   pass = false;
            }

            std::vector<uint8_t> payload;
            if (pass && (hex_k > 0 || do_parse)) {
                payload.resize(h.length);
                if (h.length > 0) {
                    ifs.read(reinterpret_cast<char*>(payload.data()), h.length);
                    if (!ifs) throw std::runtime_error("payload read failed");
                }
            } else {
                ifs.seekg(static_cast<std::streamoff>(h.length), std::ios::cur);
                if (!ifs) throw std::runtime_error("seek payload failed");
            }

            if (pass) {
                std::cout << "#" << idx
                          << " off=" << offset
                          << " type=" << h.type
                          << " ts=" << h.timestamp_ns
                          << " len=" << h.length
                          << " crc32=" << h.crc32 << "\n";

                if (hex_k > 0) hexdumpPrefix(payload, hex_k);

                if (do_parse) {
                    Record r{};
                    r.type = h.type;
                    r.timestamp = h.timestamp_ns;
                    r.length = h.length;
                    r.data = std::move(payload);
                    printParsedSummary(r);
                }

                shown++;
                if (limit > 0 && shown >= limit) break;
            }

            offset = end_off;
            idx++;
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        return 1;
    }
}

