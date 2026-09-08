#pragma once

// Forward kinematics of the G1 articulated body: joint state q(t) -> the DYNAMIC
// transforms the frame tree needs. This is NOT a frame definition (frames/ says
// which frames exist) and NOT the SE(3) math (transforms/) — it answers "given
// the current joint angles, what is T_pelvis_torso right now?".
//
// Output is a Transform (geometry layer, Eigen quaternion) — the whole FK / state
// estimator / transform_tree / deskew stack speaks Transform; the legacy Rigid is
// retired here.
//
// Today only the 3-DOF waist (pelvis -> torso) is implemented. It extends to
// pelvis -> {left,right}_foot for the contact-aided InEKF, at which point a full
// G1Model (or Pinocchio-backed FK) can absorb this. Deskew needs the composed
//   T_odom_lidar(t) = T_odom_pelvis(t) * T_pelvis_torso(q(t)) * T_torso_lidar
// so this dynamic waist term must be included — pelvis estimation alone does not
// capture the head sensors swinging with the waist.

#include "transforms/transform.hpp"

namespace kist {

// One leg's 6 joint angles (rad), in URDF chain order.
struct LegJoints {
    double hip_pitch   = 0.0;
    double hip_roll    = 0.0;
    double hip_yaw     = 0.0;
    double knee        = 0.0;
    double ankle_pitch = 0.0;
    double ankle_roll  = 0.0;
};

class G1Kinematics {
public:
    // T_pelvis_torso(q): the 3-DOF waist chain (waist_yaw / roll / pitch), from
    // the g1_29dof_mode_11 URDF link offsets. Includes the per-joint origin
    // translations, not just the rotations.
    static Transform pelvisToTorso(double waist_yaw, double waist_roll, double waist_pitch);

    // T_pelvis_foot(q): the 6-DOF leg chain to the ANKLE_ROLL link, which the first
    // estimator uses as the foot/contact body frame (the g1_29dof_mode_11 URDF has
    // no separate foot_link; contact spheres live on ankle_roll_link). No sole
    // offset is baked in — a left_sole/right_sole frame can be added later if a
    // flat-foot contact point is needed. Left and right use their OWN URDF values
    // (not a y-flip of one another); note the fixed rpy offsets on hip_roll
    // (-0.1749) and knee (+0.1749) — this is not a plain 6-axis chain.
    static Transform pelvisToLeftFoot(const LegJoints& q);
    static Transform pelvisToRightFoot(const LegJoints& q);
};

} // namespace kist
