#pragma once

// Full state of the odom->pelvis estimator (InEKF / KF): the pelvis Pose plus the
// quantities that belong to the estimator, not to a pose sample — velocity and
// IMU biases (covariance lands here later). Deskew / the transform tree only need
// .pose; keeping the rest here stops Pose from bloating into a state vector.

#include "transforms/pose.hpp"

#include <Eigen/Core>

namespace kist {

struct EstimatorState {
    Pose            pose;                                  // pelvis in odom (position + orientation + stamp)
    Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias       = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias      = Eigen::Vector3d::Zero();
    // No angular_velocity: an InEKF derives it as (gyro - gyro_bias), it is not a
    // state. covariance (Eigen fixed-size) added with the estimator.
};

} // namespace kist
