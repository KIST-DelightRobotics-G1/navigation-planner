#pragma once

// The transforms the planner's consumers read — output of LioTransformProducer, all in
// the stable odom frame. Lives next to its producer (same pairing as the readers).
//
//   T_odom_lidar  : the sensor / scan centre in odom — WOBBLES with the head sway
//   T_odom_pelvis : the robot base in odom — sway removed via the waist FK; THIS is the
//                   robot's position on the occupancy grid / for planning

#include "transforms/transform.hpp"

#include <cstdint>

namespace kist {

struct RobotTransforms {
    int64_t   stamp_ns = 0;
    Transform T_odom_lidar;    // p_odom = T_odom_lidar  * p_lidar
    Transform T_odom_pelvis;   // p_odom = T_odom_pelvis * p_pelvis
};

} // namespace kist
