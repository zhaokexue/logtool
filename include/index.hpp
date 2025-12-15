#pragma once
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <stdexcept>
#include <algorithm>

#pragma pack(push, 1)
struct IndexHeader {
    uint32_t magic = 0x31584449; // 'IDX1'
    uint16_t version = 1;
    uint16_t reserved = 0;
};

struct IndexItem {
    uint64_t timestamp_ns;
    uint64_t offset;   // 指向 RecordHeader
    uint32_t type;
};
#pragma pack(pop)

static_assert(sizeof(IndexHeader) == 8, "IndexHeader size unexpected");
static_assert(sizeof(IndexItem)   == 20,"IndexItem size unexpected");

struct IndexDB {
    std::vector<IndexItem> all;  // 全量，按 timestamp 排序
    std::unordered_map<uint32_t, std::vector<IndexItem>> by_type; // 每 type 单独有序

    void buildTypeViews() {
        by_type.clear();
        for (const auto& it : all) by_type[it.type].push_back(it);
        for (auto& kv : by_type) {
            std::sort(kv.second.begin(), kv.second.end(),
                      [](const IndexItem& a, const IndexItem& b){ return a.timestamp_ns < b.timestamp_ns; });
        }
    }

    static size_t nearestIn(const std::vector<IndexItem>& v, uint64_t target_ts) {
        if (v.empty()) throw std::runtime_error("IndexDB: empty vector");
        auto it = std::lower_bound(v.begin(), v.end(), target_ts,
            [](const IndexItem& a, uint64_t ts){ return a.timestamp_ns < ts; });
        if (it == v.begin()) return 0;
        if (it == v.end()) return v.size() - 1;
        size_t i = static_cast<size_t>(it - v.begin());
        uint64_t t1 = v[i-1].timestamp_ns, t2 = v[i].timestamp_ns;
        return (target_ts - t1 <= t2 - target_ts) ? (i-1) : i;
    }

    const IndexItem& nearest(uint64_t target_ts) const {
        return all[nearestIn(all, target_ts)];
    }

    const IndexItem* nearest(uint32_t type, uint64_t target_ts) const {
        auto it = by_type.find(type);
        if (it == by_type.end() || it->second.empty()) return nullptr;
        return &it->second[nearestIn(it->second, target_ts)];
    }
};

inline void saveIndex(const std::string& idx_path, const IndexDB& db) {
    std::ofstream ofs(idx_path, std::ios::binary);
    if (!ofs) throw std::runtime_error("saveIndex: cannot open");
    IndexHeader ih{};
    ofs.write(reinterpret_cast<const char*>(&ih), sizeof(ih));
    ofs.write(reinterpret_cast<const char*>(db.all.data()), db.all.size() * sizeof(IndexItem));
    if (!ofs) throw std::runtime_error("saveIndex: write failed");
}

inline IndexDB loadIndex(const std::string& idx_path) {
    std::ifstream ifs(idx_path, std::ios::binary);
    if (!ifs) throw std::runtime_error("loadIndex: cannot open");

    IndexHeader ih{};
    ifs.read(reinterpret_cast<char*>(&ih), sizeof(ih));
    if (!ifs) throw std::runtime_error("loadIndex: read header failed");
    if (ih.magic != 0x31584449) throw std::runtime_error("loadIndex: bad magic");
    if (ih.version != 1) throw std::runtime_error("loadIndex: unsupported version");

    // 读剩余全部为 IndexItem
    ifs.seekg(0, std::ios::end);
    auto end = ifs.tellg();
    ifs.seekg(sizeof(IndexHeader), std::ios::beg);
    auto cur = ifs.tellg();
    if (end < cur) throw std::runtime_error("loadIndex: size error");

    size_t bytes = static_cast<size_t>(end - cur);
    if (bytes % sizeof(IndexItem) != 0) throw std::runtime_error("loadIndex: corrupted size");

    IndexDB db;
    db.all.resize(bytes / sizeof(IndexItem));
    if (!db.all.empty()) {
        ifs.read(reinterpret_cast<char*>(db.all.data()), bytes);
        if (!ifs) throw std::runtime_error("loadIndex: read items failed");
    }

    std::sort(db.all.begin(), db.all.end(),
              [](const IndexItem& a, const IndexItem& b){ return a.timestamp_ns < b.timestamp_ns; });
    db.buildTypeViews();
    return db;
}

