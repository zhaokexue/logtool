#pragma once
#include <cstdint>
#include <vector>

struct Pose2D { float x{0}, y{0}, angle{0}; };
struct Imu    { float pitch{0}, roll{0}, yaw{0}; };
struct Odom   { Pose2D raw_pose; Pose2D fuse_pose; };

struct LaserScan {
    uint64_t stamp_ns{0};
    float angle_min{0};
    float angle_increment{0};
    std::vector<float> beams;
};

struct GridMap {
    int32_t width{0}, height{0};
    float resolution{0};
    float origin_x{0}, origin_y{0};
    std::vector<int8_t> data;
};

enum class RobotState : uint32_t {
    selfcheck = 0, checkdock, findwall, followwall, coverage, gohome, end
};

struct Slip { bool line_slip{false}; bool rotate_slip{false}; };

struct Record {
    uint32_t type{0};
    uint64_t timestamp{0};
    uint32_t length{0};
    std::vector<uint8_t> data;
};

// type 映射
constexpr uint32_t TYPE_STATE = 100;
constexpr uint32_t TYPE_POSE  = 101;
constexpr uint32_t TYPE_IMU   = 102;
constexpr uint32_t TYPE_ODOM  = 103;
constexpr uint32_t TYPE_SLIP  = 104;
constexpr uint32_t TYPE_MAP   = 105;
constexpr uint32_t TYPE_SCAN  = 106;

