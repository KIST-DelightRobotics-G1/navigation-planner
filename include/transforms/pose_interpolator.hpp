#pragma once

// Interpolate between two poses: position linear, orientation SLERP. Used per
// LiDAR point for deskew (thousands of calls per scan), so it is pure + cheap.
// alpha is clamped to [0,1] — this function itself never extrapolates; the
// caller (PoseHistory) enforces that target_ns lies within the bracket.

#include "transforms/pose.hpp"

namespace kist {

// a.stamp_ns <= target_ns <= b.stamp_ns expected. Result carries target_ns.
Pose interpolate(const Pose& a, const Pose& b, int64_t target_ns);

} // namespace kist
