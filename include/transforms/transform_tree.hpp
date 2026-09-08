#pragma once

// The frame tree: composes the static (mounting) and dynamic (moving) edges into
// any T_target_source(t) the rest of the system asks for. The topology is fixed by
// frame_parent() in frame_ids.hpp — every non-root frame has exactly ONE parent —
// so an edge is identified by its CHILD frame.
//
//   static edge  (torso->lidar, ...)   -> a single Transform
//   dynamic edge (odom->pelvis, ...)   -> a TransformHistory (time-interpolated)
//
// The hot path is lookupTransform(Odom, Lidar, point_stamp) for LiDAR deskew:
//   T_odom_lidar(t) = T_odom_pelvis(t) * T_pelvis_torso(t) * T_torso_lidar
// It walks up from source and target to their lowest common ancestor, so edges
// ABOVE the common ancestor (e.g. map->odom before localization exists) are not
// required. Any missing edge, or a stamp outside a dynamic edge's history, yields
// nullopt — the caller decides how to handle a gap.
//
// NOT thread-safe: producers write, consumers read. Synchronize at the owner (as
// with any shared container); the tree itself keeps no locks or threads.

#include "frames/frame_ids.hpp"
#include "transforms/transform.hpp"
#include "transforms/transform_history.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <unordered_map>

namespace kist {

class TransformTree {
public:
    // A time-invariant mounting edge. parent must be frame_parent(child); returns
    // false (and ignores the edge) on a topology mismatch.
    bool setStaticTransform(FrameId parent, FrameId child, const Transform& T);

    // A new sample of a moving edge T_parent_child at stamp_ns. parent must be
    // frame_parent(child). Samples are expected in ascending stamp order. Returns
    // false (and drops the sample) on a topology mismatch.
    bool updateTransform(FrameId parent, FrameId child,
                         const Transform& T, int64_t stamp_ns);

    // T_target_source at stamp_ns (p_target = T * p_source). nullopt if any edge on
    // the path is unset, or a dynamic edge cannot interpolate at stamp_ns.
    std::optional<Transform> lookupTransform(FrameId target, FrameId source,
                                             int64_t stamp_ns) const;

private:
    static constexpr std::size_t N = static_cast<std::size_t>(FrameId::Count);

    // T_parent_child for the edge whose child is `child`, at stamp_ns.
    std::optional<Transform> edgeTransform(FrameId child, int64_t stamp_ns) const;

    // T_ancestor_frame by composing edges from `frame` up to `ancestor` (which must
    // lie on frame's parent chain). Identity when frame == ancestor.
    std::optional<Transform> composeUp(FrameId frame, FrameId ancestor,
                                       int64_t stamp_ns) const;

    // Lowest common ancestor of a and b (always exists; both reach Map).
    static std::optional<FrameId> lowestCommonAncestor(FrameId a, FrameId b);

    std::array<std::optional<Transform>, N> static_edges_{};     // keyed by child
    std::unordered_map<int, TransformHistory> dynamic_edges_;    // keyed by int(child)
};

} // namespace kist
