#pragma once

// The STATIC edges of the frame tree — measured, fixed mounting extrinsics loaded
// from config (torso->lidar, torso->d455, d455->depth_optical). Time-varying edges
// T(t) are NOT here — each producer pushes them into the transform tree's dynamic
// history at runtime, WITH a timestamp:
//   map->odom = localization,  odom->pelvis = state estimator,
//   pelvis->torso = kinematics (waist FK).
//
// The result is a flat list of static edges so transform_tree can ingest it with:
//     for (const auto& e : ext.transforms)
//         tree.setStaticTransform(e.parent, e.child, e.T_parent_child);

#include "frames/frame_ids.hpp"
#include "transforms/transform.hpp"

#include <optional>
#include <string>
#include <vector>

namespace kist {

// One measured, fixed edge: p_parent = T_parent_child * p_child.
struct StaticTransform {
    FrameId   parent;
    FrameId   child;
    Transform T_parent_child;
};

struct SensorExtrinsics {
    std::vector<StaticTransform> transforms;
};

// Load static extrinsics from a YAML file. RPY (radians, URDF order
// R = Rz(yaw) Ry(pitch) Rx(roll)) is converted to a quaternion at load time and
// never kept as RPY. Returns nullopt and sets *err (if given) on: file/parse
// error, missing parent/child, unknown frame name, self-edge, or a duplicate
// parent->child edge.
std::optional<SensorExtrinsics> load_sensor_extrinsics(const std::string& path,
                                                       std::string* err = nullptr);

} // namespace kist
