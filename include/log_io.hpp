#pragma once
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include "log_types.hpp"

#pragma pack(push, 1)
struct FileHeader {
    uint32_t magic = 0x31474F4C; // 'LOG1'
    uint16_t version = 1;
    uint16_t flags = 0;          // bit0: has_crc32 (预留)
};

struct RecordHeader {
    uint32_t type;
    uint64_t timestamp_ns;
    uint32_t length;
    uint32_t crc32;              // flags.bit0=1 才有效；目前写0
};
#pragma pack(pop)

static_assert(sizeof(FileHeader)  == 8,  "FileHeader size unexpected");
static_assert(sizeof(RecordHeader)== 20, "RecordHeader size unexpected");

class LogWriter {
public:
    explicit LogWriter(const std::string& path) : ofs_(path, std::ios::binary) {
        if (!ofs_) throw std::runtime_error("LogWriter: cannot open");
        FileHeader fh{};
        ofs_.write(reinterpret_cast<const char*>(&fh), sizeof(fh));
        if (!ofs_) throw std::runtime_error("LogWriter: write header failed");
    }

    uint64_t writeRecord(const Record& rec) {
        // 返回该条记录的 offset（指向 RecordHeader），用于即时索引
        auto pos = ofs_.tellp();
        if (pos < 0) throw std::runtime_error("tellp failed");
        uint64_t offset = static_cast<uint64_t>(pos);

        RecordHeader h{};
        h.type = rec.type;
        h.timestamp_ns = rec.timestamp;
        h.length = rec.length;
        h.crc32 = 0;

        ofs_.write(reinterpret_cast<const char*>(&h), sizeof(h));
        if (!rec.data.empty()) ofs_.write(reinterpret_cast<const char*>(rec.data.data()), rec.data.size());
        if (!ofs_) throw std::runtime_error("LogWriter: write failed");
        return offset;
    }

private:
    std::ofstream ofs_;
};

class LogReader {
public:
    explicit LogReader(const std::string& path) : ifs_(path, std::ios::binary) {
        if (!ifs_) throw std::runtime_error("LogReader: cannot open");
        FileHeader fh{};
        ifs_.read(reinterpret_cast<char*>(&fh), sizeof(fh));
        if (!ifs_) throw std::runtime_error("LogReader: read header failed");
        if (fh.magic != 0x31474F4C) throw std::runtime_error("LogReader: bad magic");
        if (fh.version != 1) throw std::runtime_error("LogReader: unsupported version");
        flags_ = fh.flags;
    }

    bool readNext(Record& out) {
        RecordHeader h{};
        ifs_.read(reinterpret_cast<char*>(&h), sizeof(h));
        if (!ifs_) return false;

        out.type = h.type;
        out.timestamp = h.timestamp_ns;
        out.length = h.length;
        out.data.resize(h.length);

        if (h.length > 0) {
            ifs_.read(reinterpret_cast<char*>(out.data.data()), h.length);
            if (!ifs_) throw std::runtime_error("LogReader: payload read failed");
        }
        // TODO: CRC verify if flags_ & 1
        return true;
    }

    Record readAt(uint64_t offset) {
        ifs_.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
        if (!ifs_) throw std::runtime_error("LogReader: seek failed");
        Record r{};
        if (!readNext(r)) throw std::runtime_error("LogReader: readAt failed");
        return r;
    }

private:
    std::ifstream ifs_;
    uint16_t flags_{0};
};

