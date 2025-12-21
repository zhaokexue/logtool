#pragma once
#include <cstdint>
#include <cstring>
#include <vector>
#include <type_traits>
#include <stdexcept>
#include <algorithm>

#include "log_types.hpp"

// ---------------- BufferWriter (LE) ----------------
class BufferWriter {
public:
    std::vector<uint8_t> buf;

    void reserve(size_t n) { buf.reserve(n); }

    void writeBytes(const void* p, size_t n) {
        const auto* b = static_cast<const uint8_t*>(p);
        buf.insert(buf.end(), b, b + n);
    }

    template <typename T>
    void writeLE(const T& v) {
        static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
        uint8_t tmp[sizeof(T)];
        std::memcpy(tmp, &v, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        for (size_t i = 0; i < sizeof(T)/2; ++i) std::swap(tmp[i], tmp[sizeof(T)-1-i]);
#endif
        writeBytes(tmp, sizeof(T));
    }

    void writeBool(bool b) { writeLE<uint8_t>(b ? 1u : 0u); }

    template <typename T>
    void writeVectorLE(const std::vector<T>& v) {
        static_assert(std::is_trivially_copyable_v<T>, "vector element must be trivially copyable");
        writeLE<uint32_t>(static_cast<uint32_t>(v.size()));
        for (const auto& e : v) writeLE<T>(e);
    }

    void writeVectorI8(const std::vector<int8_t>& v) {
        writeLE<uint32_t>(static_cast<uint32_t>(v.size()));
        if (!v.empty()) writeBytes(v.data(), v.size());
    }
};

// ---------------- BufferReader (LE) ----------------
class BufferReader {
public:
    BufferReader(const uint8_t* data, size_t size) : p_(data), end_(data + size) {}

    size_t remaining() const { return static_cast<size_t>(end_ - p_); }

    void readBytes(void* out, size_t n) {
        if (remaining() < n) throw std::runtime_error("BufferReader underrun");
        std::memcpy(out, p_, n);
        p_ += n;
    }

    template <typename T>
    T readLE() {
        static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
        uint8_t tmp[sizeof(T)];
        readBytes(tmp, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        for (size_t i = 0; i < sizeof(T)/2; ++i) std::swap(tmp[i], tmp[sizeof(T)-1-i]);
#endif
        T v{};
        std::memcpy(&v, tmp, sizeof(T));
        return v;
    }

    bool readBool() { return readLE<uint8_t>() != 0; }

    template <typename T>
    std::vector<T> readVectorLE() {
        static_assert(std::is_trivially_copyable_v<T>, "vector element must be trivially copyable");
        uint32_t n = readLE<uint32_t>();
        std::vector<T> v;
        v.reserve(n);
        for (uint32_t i = 0; i < n; ++i) v.push_back(readLE<T>());
        return v;
    }

    std::vector<int8_t> readVectorI8() {
        uint32_t n = readLE<uint32_t>();
        if (remaining() < n) throw std::runtime_error("BufferReader underrun vector<int8_t>");
        std::vector<int8_t> v(n);
        readBytes(v.data(), n);
        return v;
    }

private:
    const uint8_t* p_{nullptr};
    const uint8_t* end_{nullptr};
};

// ---------------- serialize overloads ----------------
inline void serialize(BufferWriter& w, const Pose2D& v) { w.writeLE(v.x); w.writeLE(v.y); w.writeLE(v.angle); }
inline void serialize(BufferWriter& w, const Imu& v)    { w.writeLE(v.pitch); w.writeLE(v.roll); w.writeLE(v.yaw); }
inline void serialize(BufferWriter& w, const Odom& v)   { serialize(w, v.raw_pose); serialize(w, v.fuse_pose); }

inline void serialize(BufferWriter& w, const LaserScan& v) {
    w.writeLE(v.stamp_ns);
    w.writeLE(v.angle_min);
    w.writeLE(v.angle_increment);
    w.writeVectorLE(v.beams);
}

inline void serialize(BufferWriter& w, const GridMap& v) {
    w.writeLE(v.width); w.writeLE(v.height);
    w.writeLE(v.resolution);
    w.writeLE(v.origin_x); w.writeLE(v.origin_y);
    w.writeVectorI8(v.data);
}

inline void serialize(BufferWriter& w, const Slip& v) {
    w.writeBool(v.line_slip);
    w.writeBool(v.rotate_slip);
}

inline void serialize(BufferWriter& w, const RobotStatus& v) {
    w.writeLE(v.exception);
    w.writeLE(v.motion_state);

    w.writeBool(v.left_bumper);
    w.writeBool(v.right_bumper);
    w.writeBool(v.left_wheel_up);
    w.writeBool(v.right_wheel_up);

    w.writeBool(v.right_ir);

    w.writeLE(v.ctrl_v);
    w.writeLE(v.ctrl_w);

    w.writeBool(v.cliff_lr);
    w.writeBool(v.cliff_lf);
    w.writeBool(v.cliff_rf);
    w.writeBool(v.cliff_rr);

    w.writeBool(v.line_slip_fwd);
    w.writeBool(v.line_slip_back);

    w.writeLE(v.sonar);

    w.writeBool(v.rotate_slip_cw);
    w.writeBool(v.rotate_slip_ccw);

    w.writeLE(v.imu_yaw_vel);
    w.writeLE(v.imu_acc_x);
    w.writeLE(v.imu_acc_y);
    w.writeLE(v.imu_acc_z);

    w.writeBool(v.dock_ir1);
    w.writeBool(v.dock_ir2);
    w.writeBool(v.dock_ir3);
    w.writeBool(v.dock_ir4);
    w.writeBool(v.dock_clip_state);

    w.writeLE(v.battery_voltage);
}

inline void serialize(BufferWriter& w, RobotState v) {
    w.writeLE(static_cast<uint32_t>(v));
}

// ---------------- deserialize overloads ----------------
inline void deserialize(BufferReader& r, Pose2D& v) { v.x=r.readLE<float>(); v.y=r.readLE<float>(); v.angle=r.readLE<float>(); }
inline void deserialize(BufferReader& r, Imu& v)   { v.pitch=r.readLE<float>(); v.roll=r.readLE<float>(); v.yaw=r.readLE<float>(); }
inline void deserialize(BufferReader& r, Odom& v)  { deserialize(r, v.raw_pose); deserialize(r, v.fuse_pose); }

inline void deserialize(BufferReader& r, LaserScan& v) {
    v.stamp_ns = r.readLE<uint64_t>();
    v.angle_min = r.readLE<float>();
    v.angle_increment = r.readLE<float>();
    v.beams = r.readVectorLE<float>();
}

inline void deserialize(BufferReader& r, GridMap& v) {
    v.width = r.readLE<int32_t>();
    v.height = r.readLE<int32_t>();
    v.resolution = r.readLE<float>();
    v.origin_x = r.readLE<float>();
    v.origin_y = r.readLE<float>();
    v.data = r.readVectorI8();
}

inline void deserialize(BufferReader& r, Slip& v) {
    v.line_slip = r.readBool();
    v.rotate_slip = r.readBool();
}

inline void deserialize(BufferReader& r, RobotStatus& v) {
    v.exception = r.readLE<uint32_t>();
    v.motion_state = r.readLE<uint32_t>();

    v.left_bumper = r.readBool();
    v.right_bumper = r.readBool();
    v.left_wheel_up = r.readBool();
    v.right_wheel_up = r.readBool();

    v.right_ir = r.readBool();

    v.ctrl_v = r.readLE<float>();
    v.ctrl_w = r.readLE<float>();

    v.cliff_lr = r.readBool();
    v.cliff_lf = r.readBool();
    v.cliff_rf = r.readBool();
    v.cliff_rr = r.readBool();

    v.line_slip_fwd = r.readBool();
    v.line_slip_back = r.readBool();

    v.sonar = r.readLE<float>();

    v.rotate_slip_cw = r.readBool();
    v.rotate_slip_ccw = r.readBool();

    v.imu_yaw_vel = r.readLE<float>();
    v.imu_acc_x = r.readLE<float>();
    v.imu_acc_y = r.readLE<float>();
    v.imu_acc_z = r.readLE<float>();

    v.dock_ir1 = r.readBool();
    v.dock_ir2 = r.readBool();
    v.dock_ir3 = r.readBool();
    v.dock_ir4 = r.readBool();
    v.dock_clip_state = r.readBool();

    v.battery_voltage = r.readLE<float>();
}

inline void deserialize(BufferReader& r, RobotState& v) {
    v = static_cast<RobotState>(r.readLE<uint32_t>());
}

// ---------------- pack/unpack ----------------
template <typename T>
Record pushRecordData(uint32_t type, const T& data, uint64_t timestamp_ns) {
    Record ret;
    ret.type = type;
    ret.timestamp = timestamp_ns;

    BufferWriter w;
    w.reserve(64);
    serialize(w, data);

    ret.length = static_cast<uint32_t>(w.buf.size());
    ret.data = std::move(w.buf);
    return ret;
}

template <typename T>
T parseRecordPayload(const Record& rec) {
    BufferReader r(rec.data.data(), rec.data.size());
    T out{};
    deserialize(r, out);
    if (r.remaining() != 0) throw std::runtime_error("payload has trailing bytes (version mismatch?)");
    return out;
}

