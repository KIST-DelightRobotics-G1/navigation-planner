#include "transforms/pose_interpolator.hpp"

#include <algorithm>

namespace kist {

Pose interpolate(const Pose& a, const Pose& b, int64_t target_ns) {
    const int64_t span = b.stamp_ns - a.stamp_ns;
    double alpha = (span > 0) ? double(target_ns - a.stamp_ns) / double(span) : 0.0;
    alpha = std::clamp(alpha, 0.0, 1.0);

    Pose p;
    p.stamp_ns    = target_ns;
    p.position    = (1.0 - alpha) * a.position + alpha * b.position;
    p.orientation = a.orientation.slerp(alpha, b.orientation);
    return p;
}

} // namespace kist
