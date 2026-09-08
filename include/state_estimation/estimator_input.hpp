#pragma once

// Estimator-facing input samples — deliberately free of any Unitree / DDS type, so
// state_estimation/ and kinematics/ never depend on the robot's message layer. A
// G1 adapter fills these from LowState (joint indices below) with the MEASUREMENT
// timestamp, not the callback time.
//
// Official g1_29dof_mode_11 LowState motor indices:
//   Left  leg  0..5  : hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
//   Right leg  6..11 : hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
//   Waist     12..14 : yaw, roll, pitch

#include <Eigen/Core>

#include <cstdint>

namespace kist {

struct ImuSample {
    int64_t         stamp_ns = 0;
    Eigen::Vector3d gyro     = Eigen::Vector3d::Zero();   // rad/s, pelvis/IMU frame
    Eigen::Vector3d accel    = Eigen::Vector3d::Zero();   // m/s^2, includes gravity
};

struct LegJointSample {
    int64_t stamp_ns = 0;

    double left_hip_pitch   = 0.0;
    double left_hip_roll    = 0.0;
    double left_hip_yaw     = 0.0;
    double left_knee        = 0.0;
    double left_ankle_pitch = 0.0;
    double left_ankle_roll  = 0.0;

    double right_hip_pitch   = 0.0;
    double right_hip_roll    = 0.0;
    double right_hip_yaw     = 0.0;
    double right_knee        = 0.0;
    double right_ankle_pitch = 0.0;
    double right_ankle_roll  = 0.0;
};

struct FootContactSample {
    int64_t stamp_ns      = 0;
    bool    left_contact  = false;
    bool    right_contact = false;
};

} // namespace kist
