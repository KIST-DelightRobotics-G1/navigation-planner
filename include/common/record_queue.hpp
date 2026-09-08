#pragma once

// Bounded hand-off queue between a producer (a sensor callback thread) and
// one writer thread. This is the losslessness contract of the recorder,
// kept in exactly one place so every writer (camera blobs today, UWB /
// robot-state rows later) inherits it instead of re-implementing it:
//
//   - push() never blocks the producer: a full queue returns false and the
//     caller counts the drop (the callback thread must stay cheap).
//   - close() is drain-then-stop: pop_all() keeps handing out everything
//     already queued and returns false only once closed AND empty — so
//     every record accepted before close() reaches the writer.
//
// One producer + one consumer per instance (each recorded stream owns its
// own queue and writer thread; streams never share).

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

template <typename T>
class RecordQueue {
public:
    // (Re)arm for a recording run. Call before the writer thread starts.
    void open(size_t capacity) {
        std::lock_guard<std::mutex> lock(mutex_);
        capacity_ = capacity;
        closed_   = false;
        queue_.clear();
    }

    // Producer side. False = queue full, record NOT taken (count it as a
    // drop). Also refuses after close().
    bool push(T&& item) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_ || queue_.size() >= capacity_) return false;
            queue_.push_back(std::move(item));
        }
        cv_.notify_one();
        return true;
    }

    // Consumer side. Blocks until records arrive, then moves the whole
    // batch into `out` (previous contents discarded). Returns false only
    // when the queue is closed and fully drained — the writer's exit signal.
    bool pop_all(std::deque<T>& out) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return !queue_.empty() || closed_; });
        if (queue_.empty()) return false;   // closed_ && drained
        out.clear();
        out.swap(queue_);
        return true;
    }

    // Stop accepting; wake the consumer to drain the remainder and exit.
    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        cv_.notify_one();
    }

private:
    std::deque<T>           queue_;
    size_t                  capacity_ = 0;
    bool                    closed_   = false;
    mutable std::mutex      mutex_;
    std::condition_variable cv_;
};
