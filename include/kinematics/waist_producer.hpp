#pragma once

// Thin producer/adapter: a waist joint sample -> waist FK -> the pelvis->torso edge
// of the transform tree. Has the side effect of writing the tree (so not a "pure"
// function), but nothing more — no thread, no DDS, no ownership. Those belong to
// the runtime layer that will drive this at lowstate rate.

#include "frames/frame_ids.hpp"
#include "kinematics/g1_kinematics.hpp"
#include "transforms/transform_tree.hpp"

#include <cstdint>

namespace kist {

// Frame-neutral waist sample — no Unitree / DDS types, so kinematics/ never depends
// on the robot's message layer. A Unitree adapter fills this from LowState joints
// 12/13/14. stamp_ns MUST be the joint MEASUREMENT time (not the callback/now time):
// deskew interpolates pelvis->torso at each LiDAR point's stamp, so a stale stamp
// injects a direct phase error while the torso is moving.
struct WaistSample {
    int64_t stamp_ns = 0;
    double  yaw_rad   = 0.0;
    double  roll_rad  = 0.0;
    double  pitch_rad = 0.0;
};

// WaistSample -> FK -> tree.updateTransform(Pelvis, Torso, T_pelvis_torso(t)).
// Returns what the tree reports (false only on a topology mismatch, which cannot
// happen for this fixed edge).
bool updateWaistTransform(TransformTree& tree, const WaistSample& sample);

} // namespace kist
