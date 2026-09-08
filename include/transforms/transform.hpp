#pragma once

// Pure SE(3) math — translation + quaternion, and the minimal ops. NO timestamp,
// NO frame ids: those are metadata (see stamped_transform.hpp). Keep it this way.
//
// Convention:
//   T_A_B transforms a value expressed in frame B into frame A.
//       p_A = T_A_B * p_B
//   Composition:  T_A_C = T_A_B * T_B_C          (inner B cancels)
//   Inverse:      T_B_A = T_A_B.inverse()
//
//   T_A_B.translation = the origin of frame B expressed in frame A.
//     e.g. T_pelvis_lidar.translation = (0.10, 0, 0.70) means the LiDAR origin is
//     10 cm forward, 0 left, 70 cm up of the pelvis origin (FLU).
//
// Rotation is stored as a quaternion; never keep roll/pitch/yaw as the primary
// representation. transformPoint() applies R + t (positions); rotateVector()
// applies R only (velocities / accelerations / gravity — no translation).

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kist {

struct Transform {
    Eigen::Vector3d    translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation    = Eigen::Quaterniond::Identity();

    static Transform Identity() { return {}; }

    Eigen::Isometry3d matrix() const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear()      = rotation.normalized().toRotationMatrix();
        T.translation() = translation;
        return T;
    }

    // Position: p_A = R p_B + t.
    Eigen::Vector3d transformPoint(const Eigen::Vector3d& p) const {
        return rotation * p + translation;
    }
    // Free vector (velocity / accel / gravity): v_A = R v_B, no translation.
    Eigen::Vector3d rotateVector(const Eigen::Vector3d& v) const {
        return rotation * v;
    }

    Transform inverse() const {
        Transform out;
        out.rotation    = rotation.conjugate();
        out.translation = -(out.rotation * translation);
        return out;
    }
};

// Composition: T_A_C = T_A_B * T_B_C.
inline Transform operator*(const Transform& T_A_B, const Transform& T_B_C) {
    Transform T_A_C;
    T_A_C.rotation    = (T_A_B.rotation * T_B_C.rotation).normalized();
    T_A_C.translation = T_A_B.rotation * T_B_C.translation + T_A_B.translation;
    return T_A_C;
}

// Point transform: p_A = T_A_B * p_B.
inline Eigen::Vector3d operator*(const Transform& T_A_B, const Eigen::Vector3d& p_B) {
    return T_A_B.transformPoint(p_B);
}

} // namespace kist
