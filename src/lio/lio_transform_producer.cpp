#include "lio/lio_transform_producer.hpp"

#include "kinematics/g1_kinematics.hpp"

#include <Eigen/Geometry>

namespace kist {

namespace {
// URDF-order rpy (R = Rz(yaw) Ry(pitch) Rx(roll)) + translation -> Transform.
Transform make_transform(double x, double y, double z,
                         double roll, double pitch, double yaw) {
    Transform t;
    t.translation = Eigen::Vector3d(x, y, z);
    t.rotation = Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));
    return t;
}
} // namespace

LioTransformProducer::LioTransformProducer() {
    // Mid-360 built-in imu -> lidar scan centre (datasheet; axes aligned).
    T_lidar_imu_lidar_ = make_transform(-0.011, -0.02329, 0.04412, 0.0, 0.0, 0.0);

    // torso -> lidar mount (g1_29dof_mode_11 URDF torso_link -> mid360_link).
    // CALIBRATION: the URDF roll is pi (Mid-360 mounted upside down), but the LIO engine
    // already re-orients the sensor upright (driver roll=180), so the LIO lidar frame
    // differs from the URDF one by that correction. We start from the URDF value; verify
    // empirically — the pelvis must stay STABLE while the head sways — and adjust the
    // rotation here if it does not (likely roll -> 0 once the correction is accounted for).
    // roll set to 0 (NOT the URDF pi): the engine already corrects the upside-down mount,
    // so the LIO lidar frame is upright — the URDF roll=pi would double-flip z (pelvis
    // ended up ABOVE the lidar). pitch/translation kept from the URDF; refine empirically.
    T_torso_lidar_ = make_transform(0.0002835, 0.00003, 0.428434,
                                    0.0, 0.05112069379091391, 0.0);
}

void LioTransformProducer::step(const LioOdometry& odom, const UnitreeState& state) {
    // Waist FK: joints 12/13/14 (yaw/roll/pitch) -> T_pelvis_torso.
    const Transform T_pelvis_torso = G1Kinematics::pelvisToTorso(
        state.motors[12].q, state.motors[13].q, state.motors[14].q);

    const Transform& T_odom_lidar_imu = odom.pose.T_parent_child;      // LIO
    const Transform  T_odom_lidar  = T_odom_lidar_imu * T_lidar_imu_lidar_;
    const Transform  T_odom_torso  = T_odom_lidar * T_torso_lidar_.inverse();
    const Transform  T_odom_pelvis = T_odom_torso * T_pelvis_torso.inverse();

    RobotTransforms out;
    out.stamp_ns      = odom.pose.stamp_ns;
    out.T_odom_lidar  = T_odom_lidar;
    out.T_odom_pelvis = T_odom_pelvis;
    out_buf.SetData(out);
}

} // namespace kist
