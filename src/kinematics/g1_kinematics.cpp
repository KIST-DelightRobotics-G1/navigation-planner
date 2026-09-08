#include "kinematics/g1_kinematics.hpp"

#include <Eigen/Geometry>

namespace kist {

Transform G1Kinematics::pelvisToTorso(double waist_yaw, double waist_roll, double waist_pitch) {
    // g1_29dof_mode_11 URDF waist joints (== rev_1_0; the two differ only in the
    // drivetrain, not this chain). Each joint origin has rpy=0, so it is a pure
    // translation, and the joint adds a rotation about its axis:
    //   waist_yaw   z-axis  origin (0, 0, 0)
    //   waist_roll  x-axis  origin (-0.0039635, 0, 0.044)
    //   waist_pitch y-axis  origin (0, 0, 0)
    // T_pelvis_torso = O_yaw·Rz(yaw) · O_roll·Rx(roll) · O_pitch·Ry(pitch).
    Transform yaw;
    yaw.translation = Eigen::Vector3d(0.0, 0.0, 0.0);
    yaw.rotation    = Eigen::Quaterniond(Eigen::AngleAxisd(waist_yaw, Eigen::Vector3d::UnitZ()));

    Transform roll;
    roll.translation = Eigen::Vector3d(-0.0039635, 0.0, 0.044);
    roll.rotation    = Eigen::Quaterniond(Eigen::AngleAxisd(waist_roll, Eigen::Vector3d::UnitX()));

    Transform pitch;
    pitch.translation = Eigen::Vector3d(0.0, 0.0, 0.0);
    pitch.rotation    = Eigen::Quaterniond(Eigen::AngleAxisd(waist_pitch, Eigen::Vector3d::UnitY()));

    return yaw * roll * pitch;   // T_pelvis_torso
}

// --- leg FK helpers ----------------------------------------------------------

// A fixed URDF joint origin: translation + rpy (R = Rz(yaw) Ry(pitch) Rx(roll)).
static Transform fixed(double x, double y, double z,
                       double roll, double pitch, double yaw) {
    Transform t;
    t.translation = Eigen::Vector3d(x, y, z);
    t.rotation = Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));
    return t;
}
static Transform rotX(double a) {
    Transform t; t.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX())); return t;
}
static Transform rotY(double a) {
    Transform t; t.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY())); return t;
}
static Transform rotZ(double a) {
    Transform t; t.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ())); return t;
}

// T_pelvis_ankleroll = for each joint: fixed(origin) * rotAxis(q). g1_29dof_mode_11.
Transform G1Kinematics::pelvisToLeftFoot(const LegJoints& q) {
    return fixed(0.0,       0.064452,   -0.1027,    0.0,  0.0,    0.0) * rotY(q.hip_pitch)
         * fixed(0.0,       0.052,      -0.030465,  0.0, -0.1749, 0.0) * rotX(q.hip_roll)
         * fixed(0.025001,  0.0,        -0.12412,   0.0,  0.0,    0.0) * rotZ(q.hip_yaw)
         * fixed(-0.078273, 0.0021489,  -0.17734,   0.0,  0.1749, 0.0) * rotY(q.knee)
         * fixed(0.0,      -9.4445e-5,  -0.30001,   0.0,  0.0,    0.0) * rotY(q.ankle_pitch)
         * fixed(0.0,       0.0,        -0.017558,  0.0,  0.0,    0.0) * rotX(q.ankle_roll);
}

Transform G1Kinematics::pelvisToRightFoot(const LegJoints& q) {
    return fixed(0.0,       -0.064452,  -0.1027,    0.0,  0.0,    0.0) * rotY(q.hip_pitch)
         * fixed(0.0,       -0.052,     -0.030465,  0.0, -0.1749, 0.0) * rotX(q.hip_roll)
         * fixed(0.025001,   0.0,       -0.12412,   0.0,  0.0,    0.0) * rotZ(q.hip_yaw)
         * fixed(-0.078273, -0.0021489, -0.17734,   0.0,  0.1749, 0.0) * rotY(q.knee)
         * fixed(0.0,        9.4445e-5, -0.30001,   0.0,  0.0,    0.0) * rotY(q.ankle_pitch)
         * fixed(0.0,        0.0,       -0.017558,  0.0,  0.0,    0.0) * rotX(q.ankle_roll);
}

} // namespace kist
