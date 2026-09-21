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

// Why a Goal is not Active (valid=false). Drives the SubtaskState the controller reports when
// there is nothing to drive to: None = never commanded / idle, Cancelled = a cancel was received,
// Failed = the command could not be accepted (unsupported action / bad args).
enum class GoalDisposition : uint8_t { None, Cancelled, Failed };

struct Goal {
    float x = 0.0f, y = 0.0f;   // destination position (m); frame given by in_map
    float yaw = 0.0f;           // finish heading to align to on arrival (rad)
    bool  has_yaw = false;      // whether a finish yaw was supplied (else align is skipped)
    bool  valid = false;        // false = no active goal (robot idle / cancel)
    bool  in_map = false;       // true = x/y/yaw are in the map frame (named/catalog goal, needs
                                // map->odom); false = already odom (rviz ad-hoc), pass through

    std::string name;           // source destination name (destinations.yaml), for logging/mission
    DockConfig  dock;           // per-destination terminal behavior (align / approach / standoff)

    // Cortex subtask provenance (carried through so SubtaskState can report the active subtask).
    std::string  plan_id;       // orchestrator plan id ("" for rviz ad-hoc goals)
    uint16_t     index = 0;     // subtask index within the plan
    std::string  action;        // subtask action (nav: "move_to")

    // When valid == false, WHY (and a human-readable note) — for SubtaskState reporting.
    GoalDisposition disp = GoalDisposition::None;
    std::string     note;       // e.g. "cancelled", "unsupported: ...", FAILED reason
};

} // namespace kist
