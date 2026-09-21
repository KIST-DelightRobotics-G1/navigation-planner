#pragma once

#include <cstdint>

namespace kist {

// Base-frame velocity command for the locomotion stack — the SAME contract
// kist-gearsonic-inference consumes (its motion/nav_command.hpp -> InputHandler::nav_buf):
// vx forward (+x), vy left (+y), vyaw CCW. Sent on the wire as a geometry_msgs/Twist
// (linear.x=vx, linear.y=vy, angular.z=vyaw). Zeros = stop.
struct NavCommand {
    double vx   = 0.0;   // m/s, forward
    double vy   = 0.0;   // m/s, left
    double vyaw = 0.0;   // rad/s, CCW
};

// Robot base pose in the odom frame (what the follower steers from).
struct Pose2D {
    float x = 0.f, y = 0.f, yaw = 0.f;   // odom
};

// Follower phase (status/telemetry, not control). NoPath = a goal is set but there is no route to
// follow AND the robot is not at the goal (unreachable / not planned yet) — distinct from Arrived,
// which the no-route case would otherwise masquerade as. Idle = no active goal. The controller
// worker maps these to the SubtaskState wire status (see controller/subtask_state_publisher.hpp).
enum class FollowPhase { Driving, Aligning, Approaching, Arrived, Blocked, NoPath, Idle };

// Per-destination terminal (dock) behavior: what the follower does once it reaches the goal.
// Carried by the Goal, so each destination tunes its own docking (or turns it off). An
// ad-hoc rviz goal uses the default (align + approach both off -> just stop at arrival_tol).
struct DockConfig {
    bool  align      = false;   // rotate in place to the goal yaw on arrival
    bool  approach   = false;   // creep forward to a standoff from the front object
    float standoff_m = 0.40f;   // approach: hold this distance to the nearest lethal cell ahead
    float trigger_m  = 1.20f;   // approach: only creep if an object is within this ahead
    float speed      = 0.35f;   // approach: creep speed (m/s; >= loco floor ~0.2)
};

} // namespace kist
