#pragma once

// Axis convention for the whole navigation stack — REP-103 / FLU:
//
//   +X : forward
//   +Y : left
//   +Z : up          (right-handed)
//
//   roll  : rotation about +X
//   pitch : rotation about +Y
//   yaw   : rotation about +Z
//
// Every frame in frame_ids.hpp is FLU EXCEPT the D455 depth optical frame, which
// is RDF (+X right, +Y down, +Z forward). Note the exact meaning: the optical
// frame keeps its RDF axes — the extrinsic T_torso_depth_optical does not "make"
// it FLU, it TRANSFORMS a point expressed in RDF into the FLU target:
//   p_depth_optical (RDF)  --T_torso_depth_optical-->  p_torso (FLU)
// Each frame's own convention stands; this header just records which is which so
// a boundary assert catches a mix-up. See frame_axis_convention() in frame_ids.hpp.

namespace kist {

enum class AxisConvention {
    FLU,          // forward-left-up (map/odom/pelvis/torso/lidar, robot frames)
    OpticalRDF    // right-down-forward (RealSense optical: raw camera only)
};

inline constexpr AxisConvention kNavConvention = AxisConvention::FLU;

} // namespace kist
