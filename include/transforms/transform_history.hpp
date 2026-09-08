#pragma once

// Time history of ONE dynamic tree edge T_parent_child(t) — the geometry-layer
// counterpart of PoseHistory. It stores Transform samples (NOT Pose): a tree edge
// is a coordinate transform, not the pose of a moving body, so keeping the layers
// separate stops the estimator concept from leaking into the frame tree.
//
// lookup() interpolates (translation linear, rotation SLERP) between the two
// bracketing samples. Like PoseHistory it does NOT extrapolate: a query outside
// [front, back] or with < 2 samples returns nullopt.

#include "transforms/transform.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace kist {

class TransformHistory {
public:
    explicit TransformHistory(std::size_t max_size = 2000) : max_(max_size) {}

    // Append a sample. Stamps are expected ascending (as produced by a live edge).
    void push(int64_t stamp_ns, const Transform& T);

    // T_parent_child at stamp_ns, interpolated; nullopt if < 2 samples or the
    // stamp is outside the stored range (no extrapolation).
    std::optional<Transform> lookup(int64_t stamp_ns) const;

    std::size_t size() const { return buf_.size(); }
    bool empty() const { return buf_.empty(); }

private:
    struct Sample { int64_t stamp_ns; Transform T; };
    std::deque<Sample> buf_;
    std::size_t        max_;
};

} // namespace kist
