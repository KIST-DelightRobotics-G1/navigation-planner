#pragma once

// The output of the goal-generation / mission layer: WHERE the robot should go,
// decoupled from HOW it gets there (planner + follower). One active Goal at a
// time. A consumer feeds it to the planner (set_goal) and the follower
// (set_goal_yaw), and reports arrival back so the mission can advance.
//
// This is just the data contract; the module that produces it (destination
// lookup from destinations.yaml, external DDS command intake, arrival -> next,
// person-stop handling) lands alongside this header as the refactor proceeds.

#include "controller/nav_command.hpp"   // DockConfig (terminal dock behavior)

#include <cstdint>
#include <string>

namespace kist {

struct Goal {
    float x = 0.0f, y = 0.0f;   // destination in the world/map frame (m)
    float yaw = 0.0f;           // finish heading to align to on arrival (rad, world)
    bool  has_yaw = false;      // whether a finish yaw was supplied (else align is skipped)
    bool  valid = false;        // false = no active goal (robot idle / cancel)

    std::string name;           // source destination name (destinations.yaml), for logging/mission
    DockConfig  dock;           // per-destination terminal behavior (align / approach / standoff)
};

} // namespace kist
