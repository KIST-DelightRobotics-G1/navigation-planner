#pragma once

// The frame vocabulary — every coordinate frame in the G1 navigation tree, named
// in ONE place. No math here; just identity, parent, and axis convention, so the
// rest of the system (transform_tree, estimator, deskew) refers to frames by a
// single enum.
//
//                       map
//                        │        map -> odom            : UWB / global localization
//                      odom
//                     /    \       odom -> pelvis        : state estimator (T_odom_pelvis)
//                    /      \      odom -> base_stab     : derived (roll/pitch removed)
//                   ▼        ▼
//                pelvis   base_stabilized
//                   │
//                   │ 3-DOF waist FK                     pelvis -> torso : waist FK
//                   ▼
//                 torso
//                 /   \
//                ▼     ▼
//             lidar   d455                               torso -> lidar/d455 : static extrinsic
//                      │
//                      ▼  d455 -> depth_optical : axis convention (RDF)
//                 depth_optical
//
// base_stabilized is a DERIVED navigation frame (a sibling of pelvis under odom),
// NOT a physical link — keeping it off the pelvis chain lets the estimator's
// T_odom_pelvis drop straight into the tree with no decomposition.

#include "frames/frame_convention.hpp"

#include <optional>
#include <string_view>

namespace kist {

enum class FrameId {
    Map,
    Odom,

    // Robot links
    Pelvis,
    Torso,             // torso_link, above the 3-DOF waist; sensors mount here

    // Derived navigation frame (roll/pitch removed pelvis), sibling of pelvis
    BaseStabilized,

    // Sensors
    Lidar,             // Mid-360
    D455,              // camera mounting link
    DepthOptical,      // D455 depth optical frame (RDF axes)

    Count              // sentinel
};

inline constexpr std::string_view frame_name(FrameId f) {
    switch (f) {
        case FrameId::Map:            return "map";
        case FrameId::Odom:           return "odom";
        case FrameId::Pelvis:         return "pelvis";
        case FrameId::Torso:          return "torso";
        case FrameId::BaseStabilized: return "base_stabilized";
        case FrameId::Lidar:          return "lidar";
        case FrameId::D455:           return "d455";
        case FrameId::DepthOptical:   return "depth_optical";
        default:                      return "?";
    }
}

// Inverse of frame_name(): parse a config token into a FrameId, nullopt for an
// unknown token (the extrinsics loader rejects it). Linear scan over the enum —
// tiny fixed set, called only at load time.
inline std::optional<FrameId> frame_from_name(std::string_view s) {
    for (int i = 0; i < static_cast<int>(FrameId::Count); ++i) {
        const auto f = static_cast<FrameId>(i);
        if (frame_name(f) == s) return f;
    }
    return std::nullopt;
}

// Parent of each frame; nullopt for a root (map) / the sentinel. Optional (not
// self) so tree traversal `for (f = parent(f); f; f = parent(*f))` terminates.
inline constexpr std::optional<FrameId> frame_parent(FrameId f) {
    switch (f) {
        case FrameId::Odom:           return FrameId::Map;
        case FrameId::Pelvis:         return FrameId::Odom;
        case FrameId::BaseStabilized: return FrameId::Odom;   // sibling of pelvis
        case FrameId::Torso:          return FrameId::Pelvis;
        case FrameId::Lidar:          return FrameId::Torso;
        case FrameId::D455:           return FrameId::Torso;
        case FrameId::DepthOptical:   return FrameId::D455;
        case FrameId::Map:            return std::nullopt;    // root
        default:                      return std::nullopt;    // Count / unknown
    }
}

// Axis convention of a frame — everything is FLU except the D455 depth optical
// frame (RDF). Lets a boundary assert catch convention mix-ups.
inline constexpr AxisConvention frame_axis_convention(FrameId f) {
    switch (f) {
        case FrameId::DepthOptical: return AxisConvention::OpticalRDF;
        default:                    return AxisConvention::FLU;
    }
}

} // namespace kist
