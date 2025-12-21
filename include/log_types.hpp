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

// Rich realtime status snapshot (aligned to pose timeline in viewer)
// NOTE: This is an additive type (TYPE_STATUS). It does NOT replace TYPE_STATE/IMU/ODOM/SLIP.
struct RobotStatus {
    // top bar
    uint32_t exception{0};
    uint32_t motion_state{0};

    // bumper / wheel-up
    bool left_bumper{false};
    bool right_bumper{false};
    bool left_wheel_up{false};
    bool right_wheel_up{false};

    // wall follow IR (right)
    bool right_ir{false};

    // commanded velocity
    float ctrl_v{0.0f};      // m/s
    float ctrl_w{0.0f};      // rad/s

    // cliff IR (LR, LF, RF, RR)
    bool cliff_lr{false};
    bool cliff_lf{false};
    bool cliff_rf{false};
    bool cliff_rr{false};

    // line slip (forward/back)
    bool line_slip_fwd{false};
    bool line_slip_back{false};

    // sonar distance (m)
    float sonar{0.0f};

    // rotate slip (cw/ccw)
    bool rotate_slip_cw{false};
    bool rotate_slip_ccw{false};

    // imu derived
    float imu_yaw_vel{0.0f}; // rad/s
    float imu_acc_x{0.0f};
    float imu_acc_y{0.0f};
    float imu_acc_z{0.0f};

    // docking
    bool dock_ir1{false};
    bool dock_ir2{false};
    bool dock_ir3{false};
    bool dock_ir4{false};
    bool dock_clip_state{false};

    float battery_voltage{0.0f};
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
constexpr uint32_t TYPE_STATUS= 107;

