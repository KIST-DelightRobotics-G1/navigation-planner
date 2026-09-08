#pragma once

namespace kist {

// Base-frame velocity command for the locomotion stack. This is the SAME
// contract kist-gearsonic-inference consumes (its motion/nav_command.hpp ->
// InputHandler::nav_buf): vx forward, vy left, vyaw CCW. Sent on the wire as a
// ROS geometry_msgs/Twist (linear.x=vx, linear.y=vy, angular.z=vyaw).
//
// Producer contract: publish continuously (~20Hz) while following a path; publish
// zeros when arrived / no path / halted (= stop); stop publishing when inactive
// so the consumer's buffer goes stale and it falls back to manual.
struct NavCommand {
    double vx   = 0.0;   // m/s, forward (+x)
    double vy   = 0.0;   // m/s, left (+y)
    double vyaw = 0.0;   // rad/s, counter-clockwise
};

} // namespace kist
