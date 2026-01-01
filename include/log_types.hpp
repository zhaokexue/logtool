#pragma once
#include <cstdint>
#include <vector>

enum class CleanState : uint32_t {
    selfcheck = 0, checkdock, findwall, followwall, coverage, gohome, end
};

enum class ExceptionCode : uint32_t {
    low_power = 0, nai_fail, go_home_fail, end
};

enum class MotionState : uint32_t {
    rotate = 0, line, dock, cpp, npp, end
};

struct Pose2D { float x{0}, y{0}, angle{0}; };
struct Imu    { float pitch{0}, roll{0}, yaw{0}, imu_acc_x{0.0f}, imu_acc_y{0.0f}, imu_acc_z{0.0f}, imu_yaw_vel{0.0f}; };
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

struct Pose2DSet{
    std::vector<Pose2D> pose_set;
};

struct SensorData {
    // commanded velocity
    float ctrl_v{0.0f};      // m/s
    float ctrl_w{0.0f};      // rad/s

    // line slip (forward/back)
    bool line_slip_fwd{false};
    bool line_slip_back{false};
    // rotate slip (cw/ccw)
    bool rotate_slip_cw{false};
    bool rotate_slip_ccw{false};

    // odom
    Odom odom;

    // imu derived
    Imu imu;

    // bumper / wheel-up
    bool left_bumper{false};
    bool right_bumper{false};
    bool left_wheel_up{false};
    bool right_wheel_up{false};

    // wall follow IR (right)
    bool right_ir{false};
    // sonar distance (m)
    float sonar{0.0f};

    // cliff IR (LR, LF, RF, RR)
    bool cliff_lr{false};
    bool cliff_lf{false};
    bool cliff_rf{false};
    bool cliff_rr{false};

    // docking
    bool dock_ir1{false};
    bool dock_ir2{false};
    bool dock_ir3{false};
    bool dock_ir4{false};
    bool dock_clip_state{false};

    float battery_voltage{0.0f};
};

struct Record {
    uint32_t type{0};
    uint64_t timestamp{0};
    uint32_t length{0};
    std::vector<uint8_t> data;
};

// type 映射
constexpr uint32_t TYPE_CLEAN_STATE = 99;          // 频率：5HZ      清扫状态:CleanState
constexpr uint32_t TYPE_EXCEPTION_DATA = 100;      // 频率：5HZ      异常码:ExceptionCode
constexpr uint32_t TYPE_MOTION_STATE = 101;        // 频率：5HZ      运动状态:MotionState
constexpr uint32_t TYPE_SENSOR_DATA = 102;         // 频率：50HZ     传感器数据:SensorData

constexpr uint32_t TYPE_GRID_MAP = 103;            // 频率：1HZ      清扫的地图:GridMap
constexpr uint32_t TYPE_EXPROATION_GRID_MAP = 104; // 频率：1HZ      探索的地图:GridMap

constexpr uint32_t TYPE_ROBOT_LIDAR = 105;         // 频率：6HZ      单帧雷达数据:LaserScan
constexpr uint32_t TYPE_ROBOT_POSE = 106;          // 频率：6HZ      雷达POSE:Pose2D
constexpr uint32_t TYPE_ROBOT_REALTIME_POSE = 107; // 频率：20HZ     机器人实时POSE:Pose2D

constexpr uint32_t TYPE_GLOBAL_PATH_SET = 108;     // 频率：<5HZ 不定 全局路径点集:Pose2DSet
constexpr uint32_t TYPE_LOCAL_PATH_SET = 109;      // 频率：<5HZ 不定 局部路径点集:Pose2DSet

constexpr uint32_t TYPE_DOOR_POINT_SET = 110;      // 频率：<5HZ 不定 门的点集:Pose2DSet
constexpr uint32_t TYPE_PLAN_POINT_SET = 111;      // 频率：<5HZ 不定 规划点集:Pose2DSet
constexpr uint32_t TYPE_COVER_POINT_SET = 112;     // 频率：<5HZ 不定 覆盖点集:Pose2DSet
constexpr uint32_t TYPE_CHAIN_LINE_SET = 113;      // 频率：<5HZ 不定 链条点集:Pose2DSet
constexpr uint32_t TYPE_PARTICLE_SET = 114;        // 频率：<5HZ 不定 粒子点集:Pose2DSet
constexpr uint32_t TYPE_COVERAGE_AREA_SET = 115;   // 频率：<5HZ 不定 覆盖区域点集:Pose2DSet

constexpr uint32_t TYPE_POINT_DEBUG1 = 116;        // 频率：<5HZ 不定 调试点集1:Pose2DSet
constexpr uint32_t TYPE_POINT_DEBUG2 = 117;        // 频率：<5HZ 不定 调试点集2:Pose2DSet
constexpr uint32_t TYPE_POINT_DEBUG3 = 118;        // 频率：<5HZ 不定 调试点集3:Pose2DSet
