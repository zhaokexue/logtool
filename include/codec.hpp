#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>
#include <type_traits>
#include <stdexcept>

#include "log_types.hpp"

// =========================================
// Minimal binary codec for log payloads
// =========================================
// We encode fields explicitly (not raw struct memcpy) to avoid compiler-
// dependent padding/layout issues.

inline void appendBytes(std::vector<uint8_t>& out, const void* data, size_t n) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
    out.insert(out.end(), p, p + n);
}

template <class T>
inline void appendScalar(std::vector<uint8_t>& out, const T& v) {
    static_assert(std::is_trivially_copyable_v<T>, "appendScalar requires trivially copyable T");
    appendBytes(out, &v, sizeof(T));
}

template <class T>
inline T readScalar(const std::vector<uint8_t>& in, size_t& off) {
    static_assert(std::is_trivially_copyable_v<T>, "readScalar requires trivially copyable T");
    if (off + sizeof(T) > in.size()) throw std::runtime_error("codec: buffer underrun");
    T v{};
    std::memcpy(&v, in.data() + off, sizeof(T));
    off += sizeof(T);
    return v;
}

// ---------- std::vector<T> helpers (for trivially copyable element types) ----------

template <class T>
inline void appendVector(std::vector<uint8_t>& out, const std::vector<T>& v) {
    static_assert(std::is_trivially_copyable_v<T>, "appendVector requires trivially copyable T");
    const uint32_t n = static_cast<uint32_t>(v.size());
    appendScalar(out, n);
    if (!v.empty()) appendBytes(out, v.data(), v.size() * sizeof(T));
}

template <class T>
inline std::vector<T> readVector(const std::vector<uint8_t>& in, size_t& off) {
    static_assert(std::is_trivially_copyable_v<T>, "readVector requires trivially copyable T");
    const uint32_t n = readScalar<uint32_t>(in, off);
    const size_t bytes = static_cast<size_t>(n) * sizeof(T);
    if (off + bytes > in.size()) throw std::runtime_error("codec: vector overrun");
    std::vector<T> v;
    v.resize(n);
    if (bytes) std::memcpy(v.data(), in.data() + off, bytes);
    off += bytes;
    return v;
}

// =========================================
// enums (stored as uint32_t)
inline void serializeToBytes(std::vector<uint8_t>& out, const CleanState& e){ const uint32_t v = static_cast<uint32_t>(e); appendScalar(out, v); }
inline void serializeToBytes(std::vector<uint8_t>& out, const ExceptionCode& e){ const uint32_t v = static_cast<uint32_t>(e); appendScalar(out, v); }
inline void serializeToBytes(std::vector<uint8_t>& out, const MotionState& e){ const uint32_t v = static_cast<uint32_t>(e); appendScalar(out, v); }

// Per-type serialization
// =========================================


inline void serializeToBytes(std::vector<uint8_t>& out, const Pose2D& p) {
    appendScalar(out, p.x);
    appendScalar(out, p.y);
    appendScalar(out, p.angle);
}
inline Pose2D parseFromBytesPose2D(const std::vector<uint8_t>& in, size_t& off) {
    Pose2D p;
    p.x = readScalar<float>(in, off);
    p.y = readScalar<float>(in, off);
    p.angle = readScalar<float>(in, off);
    return p;
}

inline void serializeToBytes(std::vector<uint8_t>& out, const Imu& imu) {
    appendScalar(out, imu.pitch);
    appendScalar(out, imu.roll);
    appendScalar(out, imu.yaw);
    appendScalar(out, imu.imu_acc_x);
    appendScalar(out, imu.imu_acc_y);
    appendScalar(out, imu.imu_acc_z);
    appendScalar(out, imu.imu_yaw_vel);
}
inline Imu parseFromBytesImu(const std::vector<uint8_t>& in, size_t& off) {
    Imu imu;
    imu.pitch = readScalar<float>(in, off);
    imu.roll = readScalar<float>(in, off);
    imu.yaw = readScalar<float>(in, off);
    imu.imu_acc_x = readScalar<float>(in, off);
    imu.imu_acc_y = readScalar<float>(in, off);
    imu.imu_acc_z = readScalar<float>(in, off);
    imu.imu_yaw_vel = readScalar<float>(in, off);
    return imu;
}

inline void serializeToBytes(std::vector<uint8_t>& out, const Odom& o) {
    serializeToBytes(out, o.raw_pose);
    serializeToBytes(out, o.fuse_pose);
}
inline Odom parseFromBytesOdom(const std::vector<uint8_t>& in, size_t& off) {
    Odom o;
    o.raw_pose = parseFromBytesPose2D(in, off);
    o.fuse_pose = parseFromBytesPose2D(in, off);
    return o;
}

inline void serializeToBytes(std::vector<uint8_t>& out, const LaserScan& s) {
    appendScalar(out, s.stamp_ns);
    appendScalar(out, s.angle_min);
    appendScalar(out, s.angle_increment);
    appendVector(out, s.beams);
}
inline LaserScan parseFromBytesLaserScan(const std::vector<uint8_t>& in, size_t& off) {
    LaserScan s;
    s.stamp_ns = readScalar<uint64_t>(in, off);
    s.angle_min = readScalar<float>(in, off);
    s.angle_increment = readScalar<float>(in, off);
    s.beams = readVector<float>(in, off);
    return s;
}

inline void serializeToBytes(std::vector<uint8_t>& out, const GridMap& m) {
    appendScalar(out, m.width);
    appendScalar(out, m.height);
    appendScalar(out, m.resolution);
    appendScalar(out, m.origin_x);
    appendScalar(out, m.origin_y);
    appendVector(out, m.data);
}
inline GridMap parseFromBytesGridMap(const std::vector<uint8_t>& in, size_t& off) {
    GridMap m;
    m.width = readScalar<int32_t>(in, off);
    m.height = readScalar<int32_t>(in, off);
    m.resolution = readScalar<float>(in, off);
    m.origin_x = readScalar<float>(in, off);
    m.origin_y = readScalar<float>(in, off);
    m.data = readVector<int8_t>(in, off);
    return m;
}

inline void serializeToBytes(std::vector<uint8_t>& out, const Pose2DSet& s) {
    const uint32_t n = static_cast<uint32_t>(s.pose_set.size());
    appendScalar(out, n);
    for (const auto& p : s.pose_set) serializeToBytes(out, p);
}
inline Pose2DSet parseFromBytesPose2DSet(const std::vector<uint8_t>& in, size_t& off) {
    Pose2DSet s;
    const uint32_t n = readScalar<uint32_t>(in, off);
    s.pose_set.reserve(n);
    for (uint32_t i = 0; i < n; ++i) s.pose_set.push_back(parseFromBytesPose2D(in, off));
    return s;
}

inline void serializeToBytes(std::vector<uint8_t>& out, const SensorData& d) {
    appendScalar(out, d.ctrl_v);
    appendScalar(out, d.ctrl_w);

    appendScalar(out, static_cast<uint8_t>(d.line_slip_fwd ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.line_slip_back ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.rotate_slip_cw ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.rotate_slip_ccw ? 1 : 0));

    serializeToBytes(out, d.odom);
    serializeToBytes(out, d.imu);

    appendScalar(out, static_cast<uint8_t>(d.left_bumper ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.right_bumper ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.left_wheel_up ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.right_wheel_up ? 1 : 0));

    appendScalar(out, static_cast<uint8_t>(d.right_ir ? 1 : 0));
    appendScalar(out, d.sonar);

    appendScalar(out, static_cast<uint8_t>(d.cliff_lr ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.cliff_lf ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.cliff_rf ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.cliff_rr ? 1 : 0));

    appendScalar(out, static_cast<uint8_t>(d.dock_ir1 ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.dock_ir2 ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.dock_ir3 ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.dock_ir4 ? 1 : 0));
    appendScalar(out, static_cast<uint8_t>(d.dock_clip_state ? 1 : 0));

    appendScalar(out, d.battery_voltage);
}
inline SensorData parseFromBytesSensorData(const std::vector<uint8_t>& in, size_t& off) {
    SensorData d;
    d.ctrl_v = readScalar<float>(in, off);
    d.ctrl_w = readScalar<float>(in, off);

    d.line_slip_fwd = (readScalar<uint8_t>(in, off) != 0);
    d.line_slip_back = (readScalar<uint8_t>(in, off) != 0);
    d.rotate_slip_cw = (readScalar<uint8_t>(in, off) != 0);
    d.rotate_slip_ccw = (readScalar<uint8_t>(in, off) != 0);

    d.odom = parseFromBytesOdom(in, off);
    d.imu = parseFromBytesImu(in, off);

    d.left_bumper = (readScalar<uint8_t>(in, off) != 0);
    d.right_bumper = (readScalar<uint8_t>(in, off) != 0);
    d.left_wheel_up = (readScalar<uint8_t>(in, off) != 0);
    d.right_wheel_up = (readScalar<uint8_t>(in, off) != 0);

    d.right_ir = (readScalar<uint8_t>(in, off) != 0);
    d.sonar = readScalar<float>(in, off);

    d.cliff_lr = (readScalar<uint8_t>(in, off) != 0);
    d.cliff_lf = (readScalar<uint8_t>(in, off) != 0);
    d.cliff_rf = (readScalar<uint8_t>(in, off) != 0);
    d.cliff_rr = (readScalar<uint8_t>(in, off) != 0);

    d.dock_ir1 = (readScalar<uint8_t>(in, off) != 0);
    d.dock_ir2 = (readScalar<uint8_t>(in, off) != 0);
    d.dock_ir3 = (readScalar<uint8_t>(in, off) != 0);
    d.dock_ir4 = (readScalar<uint8_t>(in, off) != 0);
    d.dock_clip_state = (readScalar<uint8_t>(in, off) != 0);

    d.battery_voltage = readScalar<float>(in, off);
    return d;
}

// ---------- enum helpers ----------
template <class E>
inline void serializeEnum(std::vector<uint8_t>& out, E e) {
    using U = std::underlying_type_t<E>;
    appendScalar(out, static_cast<U>(e));
}

template <class E>
inline E parseEnum(const std::vector<uint8_t>& in, size_t& off) {
    using U = std::underlying_type_t<E>;
    U v = readScalar<U>(in, off);
    return static_cast<E>(v);
}

// =========================================
// API: serialize/parse payload by type
// =========================================

template <typename T>
inline std::vector<uint8_t> serializePayload(const T& obj) {
    std::vector<uint8_t> out;
    out.reserve(256);
    serializeToBytes(out, obj);
    return out;
}

// Generic parseRecordPayload<T>(Record)
// Record is defined in log_types.hpp and populated by LogReader.

template <typename T>
inline T parsePayload(const std::vector<uint8_t>& bytes) {
    size_t off = 0;
    if constexpr (std::is_same_v<T, Pose2D>) {
        return parseFromBytesPose2D(bytes, off);
    } else if constexpr (std::is_same_v<T, Imu>) {
        return parseFromBytesImu(bytes, off);
    } else if constexpr (std::is_same_v<T, Odom>) {
        return parseFromBytesOdom(bytes, off);
    } else if constexpr (std::is_same_v<T, LaserScan>) {
        return parseFromBytesLaserScan(bytes, off);
    } else if constexpr (std::is_same_v<T, GridMap>) {
        return parseFromBytesGridMap(bytes, off);
    } else if constexpr (std::is_same_v<T, Pose2DSet>) {
        return parseFromBytesPose2DSet(bytes, off);
    } else if constexpr (std::is_same_v<T, SensorData>) {
        return parseFromBytesSensorData(bytes, off);
    } else if constexpr (std::is_same_v<T, CleanState>) {
        return parseEnum<CleanState>(bytes, off);
    } else if constexpr (std::is_same_v<T, ExceptionCode>) {
        return parseEnum<ExceptionCode>(bytes, off);
    } else if constexpr (std::is_same_v<T, MotionState>) {
        return parseEnum<MotionState>(bytes, off);
    } else {
        static_assert(sizeof(T) == 0, "parsePayload: unsupported type");
    }
}

template <typename T>
inline T parseRecordPayload(const Record& r) {
    return parsePayload<T>(r.data);
}
