#pragma once

// Pose of a moving body at one instant — position + orientation + timestamp,
// nothing more.
//
// A Pose carries NO frame identifiers: its reference/body-frame semantics are
// owned by the producer/container (e.g. a PelvisPoseHistory stores the pose of
// pelvis expressed in odom). Velocity, IMU bias, and covariance are estimator
// state — see state_estimation/estimator_state.hpp — and deliberately do NOT
// live here, so Pose stays a light sample type for PoseHistory / interpolation.
//
// Numerically a Pose and a Transform hold the same SE(3) value; the difference is
// semantic (a moving body's state vs a coordinate operator). Convert with the
// free function toTransform() at the boundary — Pose does not know the transform
// layer.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>

namespace kist {

struct Pose {
    int64_t            stamp_ns    = 0;
    Eigen::Vector3d    position    = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

} // namespace kist
