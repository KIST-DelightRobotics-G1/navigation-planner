#pragma once

// StampRing — a tiny thread-safe ring of the last N timestamped values, with a
// nearest-by-stamp lookup. It exists so a consumer can pair a specific input with
// the pose that belongs to it (its stamp), not merely the latest pose.
//
// Why the mapping stage needs it: the registered scan and the odometry arrive on
// two different LIO receive threads. Ray-carving needs the ray ORIGIN = the sensor
// pose AT THE SCAN'S STAMP; grabbing "latest" can pick a neighbouring LIO cycle and
// smear the free-space carve. The producer pushes each RobotTransforms here; the
// mapping worker looks up nearest(cloud.stamp_ns).
//
// T must expose an `int64_t stamp_ns` member. push() overwrites the oldest slot;
// nearest() scans the (small, fixed) ring under the lock and returns a copy.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <optional>

namespace kist {

template <typename T, std::size_t N>
class StampRing {
    static_assert(N > 0, "StampRing needs capacity > 0");

public:
    void push(const T& v) {
        std::lock_guard<std::mutex> lk(mutex_);
        buf_[head_] = v;
        head_ = (head_ + 1) % N;
        if (count_ < N) ++count_;
    }

    // The stored value whose stamp is closest to stamp_ns, or nullopt if the ring is
    // empty or the closest is farther than max_dt_ns (no pose for that scan).
    std::optional<T> nearest(int64_t stamp_ns, int64_t max_dt_ns) const {
        std::lock_guard<std::mutex> lk(mutex_);
        const T*  best   = nullptr;
        int64_t   best_dt = 0;
        for (std::size_t i = 0; i < count_; ++i) {
            const int64_t dt = std::llabs(buf_[i].stamp_ns - stamp_ns);
            if (!best || dt < best_dt) { best = &buf_[i]; best_dt = dt; }
        }
        if (!best || best_dt > max_dt_ns) return std::nullopt;
        return *best;
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return count_;
    }

private:
    mutable std::mutex mutex_;
    std::array<T, N>   buf_{};
    std::size_t        head_  = 0;
    std::size_t        count_ = 0;
};

} // namespace kist
