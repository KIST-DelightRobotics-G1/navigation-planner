#pragma once

// Time-ordered pose buffer for LiDAR deskew: the estimator pushes poses (newest
// at the back, non-decreasing stamps), and each scan point queries the pose at
// its own capture time.
//
// Extrapolation policy (deliberate, first impl): FORBIDDEN. A query outside the
// buffered [front, back] time range returns nullopt — a point captured outside
// the pose window is dropped rather than guessed. Loosen only with a real reason.

#include "transforms/pose.hpp"

#include <cstddef>
#include <deque>
#include <optional>

namespace kist {

class PoseHistory {
public:
    explicit PoseHistory(size_t max_size = 2000) : max_(max_size) {}

    void push(const Pose& p);   // append; assumes non-decreasing stamp_ns

    // SLERP-interpolated pose at stamp_ns, or nullopt if out of range / too few
    // samples (no extrapolation).
    std::optional<Pose> getInterpolated(int64_t stamp_ns) const;

    bool   empty() const { return buf_.empty(); }
    size_t size()  const { return buf_.size(); }

private:
    std::deque<Pose> buf_;
    size_t           max_;
};

} // namespace kist
