#pragma once

// MapOdom — the map->odom transform, the relocalizer's output and the frame-tree's map->odom
// edge (p_map = T_map_odom * p_odom). Plus goal_to_odom(): convert a map-frame Goal into odom
// using the current estimate, so the odom-based planner/controller can chase a FIXED map-frame
// destination that stays put across LIO drift / boot origin.

#include "goal_generation/goal.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace kist {

struct MapOdom {
    Eigen::Matrix4f T_map_odom = Eigen::Matrix4f::Identity();   // p_map = T_map_odom * p_odom
};

// map-frame Goal -> odom-frame Goal:  p_odom = T_map_odom^-1 * p_map,  yaw_odom = yaw_map - yaw(T_map_odom).
// An odom goal (g.in_map == false, e.g. an rviz ad-hoc click) passes through unchanged.
inline Goal goal_to_odom(const Goal& g, const MapOdom& mo) {
    if (!g.in_map) return g;                                    // already odom
    const Eigen::Matrix4f T_om = mo.T_map_odom.inverse();      // T_odom_map
    const Eigen::Vector4f p_odom = T_om * Eigen::Vector4f(g.x, g.y, 0.f, 1.f);
    const float dyaw = std::atan2(T_om(1, 0), T_om(0, 0));     // yaw of T_odom_map (= -yaw(T_map_odom))
    Goal o = g;
    o.x = p_odom.x();
    o.y = p_odom.y();
    o.yaw = g.yaw + dyaw;                                       // yaw_map rotated into odom
    o.in_map = false;
    return o;
}

} // namespace kist
