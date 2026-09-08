#include "transforms/pose_history.hpp"

#include "transforms/pose_interpolator.hpp"

#include <algorithm>

namespace kist {

void PoseHistory::push(const Pose& p) {
    buf_.push_back(p);
    while (buf_.size() > max_)
        buf_.pop_front();
}

std::optional<Pose> PoseHistory::getInterpolated(int64_t stamp_ns) const {
    if (buf_.size() < 2) return std::nullopt;
    if (stamp_ns < buf_.front().stamp_ns || stamp_ns > buf_.back().stamp_ns)
        return std::nullopt;                              // no extrapolation

    // First sample with stamp >= target (buf_ is ascending by stamp).
    const auto it = std::lower_bound(
        buf_.begin(), buf_.end(), stamp_ns,
        [](const Pose& p, int64_t s) { return p.stamp_ns < s; });

    if (it == buf_.begin()) return *it;                   // exactly at the front
    return interpolate(*(it - 1), *it, stamp_ns);         // bracket [it-1, it]
}

} // namespace kist
