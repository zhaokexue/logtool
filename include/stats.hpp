#pragma once
#include <cstdint>
#include <unordered_map>
#include <limits>
#include <vector>
#include <algorithm>
#include <iostream>

struct TypeStats {
    uint64_t count{0};
    uint64_t total_len{0};
    uint32_t min_len{std::numeric_limits<uint32_t>::max()};
    uint32_t max_len{0};

    uint64_t first_ts{0};
    uint64_t last_ts{0};

    // 用于更稳的 fps：统计相邻 dt（ns）
    std::vector<uint64_t> dts;
};

inline void updateStats(std::unordered_map<uint32_t, TypeStats>& m,
                        uint32_t type, uint64_t ts, uint32_t len) {
    auto& s = m[type];
    if (s.count == 0) {
        s.first_ts = ts;
        s.last_ts = ts;
    } else {
        if (ts >= s.last_ts) s.dts.push_back(ts - s.last_ts);
        s.last_ts = std::max(s.last_ts, ts);
        s.first_ts = std::min(s.first_ts, ts);
    }
    s.count++;
    s.total_len += len;
    s.min_len = std::min(s.min_len, len);
    s.max_len = std::max(s.max_len, len);
}

inline double robustFpsFromDts(std::vector<uint64_t> dts) {
    if (dts.empty()) return 0.0;
    // 去掉极端值：用中位数 dt 估 fps
    std::sort(dts.begin(), dts.end());
    uint64_t med = dts[dts.size()/2];
    if (med == 0) return 0.0;
    return 1e9 / static_cast<double>(med);
}

inline void printStats(const std::unordered_map<uint32_t, TypeStats>& m) {
    std::vector<uint32_t> keys;
    keys.reserve(m.size());
    for (auto& kv : m) keys.push_back(kv.first);
    std::sort(keys.begin(), keys.end());

    std::cout << "type,count,avg_len,min_len,max_len,fps(median_dt)\n";
    for (auto type : keys) {
        const auto& s = m.at(type);
        double avg = s.count ? (double)s.total_len / (double)s.count : 0.0;
        double fps = robustFpsFromDts(s.dts);
        std::cout << type << "," << s.count << "," << avg << ","
                  << (s.min_len==std::numeric_limits<uint32_t>::max()?0:s.min_len) << ","
                  << s.max_len << "," << fps << "\n";
    }
}

