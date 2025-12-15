#pragma once
#include <vector>
#include <mutex>
#include <condition_variable>
#include <optional>

template <typename T>
class RingBuffer {
public:
    explicit RingBuffer(size_t capacity) : cap_(capacity), buf_(capacity) {}

    void push(T v) {
        std::unique_lock<std::mutex> lk(m_);
        buf_[head_] = std::move(v);
        head_ = (head_ + 1) % cap_;
        if (size_ < cap_) {
            ++size_;
        } else {
            // 覆盖最旧
            tail_ = (tail_ + 1) % cap_;
        }
        cv_.notify_one();
    }

    // 阻塞等待至少有一个元素（用于读线程->UI线程交付）
    T waitPop() {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&]{ return size_ > 0; });
        T v = std::move(buf_[tail_]);
        tail_ = (tail_ + 1) % cap_;
        --size_;
        return v;
    }

    // 非阻塞取最新一个（适合 UI：每次拖拽后只需要最近样本）
    std::optional<T> latest() const {
        std::lock_guard<std::mutex> lk(m_);
        if (size_ == 0) return std::nullopt;
        size_t idx = (head_ + cap_ - 1) % cap_;
        return buf_[idx];
    }

    size_t size() const { std::lock_guard<std::mutex> lk(m_); return size_; }

private:
    size_t cap_{0};
    mutable std::mutex m_;
    mutable std::condition_variable cv_;
    std::vector<T> buf_;
    size_t head_{0}, tail_{0}, size_{0};
};

