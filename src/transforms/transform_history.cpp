#include "transforms/transform_history.hpp"

#include <algorithm>

namespace kist {

// Interpolate a tree edge: translation linear, rotation SLERP. alpha in [0,1].
static Transform interp(const Transform& a, const Transform& b, double alpha) {
    Transform out;
    out.translation = a.translation + alpha * (b.translation - a.translation);
    out.rotation    = a.rotation.slerp(alpha, b.rotation);
    return out;
}

void TransformHistory::push(int64_t stamp_ns, const Transform& T) {
    buf_.push_back({stamp_ns, T});
    while (buf_.size() > max_)
        buf_.pop_front();
}

std::optional<Transform> TransformHistory::lookup(int64_t stamp_ns) const {
    if (buf_.size() < 2) return std::nullopt;
    if (stamp_ns < buf_.front().stamp_ns || stamp_ns > buf_.back().stamp_ns)
        return std::nullopt;                               // no extrapolation

    // First sample with stamp >= target (buf_ is ascending by stamp).
    const auto it = std::lower_bound(
        buf_.begin(), buf_.end(), stamp_ns,
        [](const Sample& s, int64_t t) { return s.stamp_ns < t; });

    if (it == buf_.begin()) return it->T;                  // exactly at the front
    const Sample& lo = *(it - 1);
    const Sample& hi = *it;
    if (hi.stamp_ns == lo.stamp_ns) return hi.T;           // guard zero interval
    const double alpha =
        double(stamp_ns - lo.stamp_ns) / double(hi.stamp_ns - lo.stamp_ns);
    return interp(lo.T, hi.T, alpha);
}

} // namespace kist
