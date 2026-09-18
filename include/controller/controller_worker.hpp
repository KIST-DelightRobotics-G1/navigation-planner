#pragma once

// ControllerWorker — the control thread (~20 Hz): gather the freshest inputs (path, pose,
// costmap, goal), drive LocalController (pure logic), then EITHER publish the command as a Twist
// (drive_enabled -> robot moves) OR just preview it. Owns its thread + loop; the control DECISION
// stays in LocalController. Stale inputs (>500 ms) are nulled so the controller stops safely.

#include "common/data_buffer.hpp"
#include "controller/local_controller.hpp"
#include "controller/nav_command.hpp"
#include "controller/nav_command_publisher.hpp"
#include "controller/nav_status_publisher.hpp"
#include "lio/lio_transform_producer.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"
#include "route_planner/planner/astar_planner/path.hpp"
#include "system/goal_source.hpp"

#include <atomic>
#include <string>
#include <thread>

namespace kist {

class ControllerWorker {
public:
    ~ControllerWorker() { stop(); }

    // `status_pub` publishes the navigation state on rt/kist/nav/status. `arrival_hold_s` is the
    // debounce: the robot must hold at the goal (phase Arrived) this long before ARRIVED is emitted.
    void start(DataBuffer<Path>& path_buf, LioTransformProducer& prod, DataBuffer<Costmap>& costmap_buf,
               GoalSource& goals, DataBuffer<NavCommand>& cmd_buf, NavCommandPublisher& pub,
               NavStatusPublisher& status_pub, bool drive_enabled, const FollowConfig& fc,
               double arrival_hold_s = 1.0);
    void stop();

private:
    void run();

    // Map the follower phase + goal to a published NavState, DEBOUNCING ARRIVED (must hold for
    // arrival_hold_s). Once confirmed, the goal is consumed (notify_arrived) and this reports a
    // PERSISTENT ARRIVED (holding_) until a new goal arrives. `dt` = seconds since the last call.
    NavState resolve_state(bool have_goal, const std::string& name, FollowPhase phase, double dt);

    LocalController          ctrl_;
    DataBuffer<Path>*        path_buf_    = nullptr;
    LioTransformProducer*    prod_        = nullptr;
    DataBuffer<Costmap>*     costmap_buf_ = nullptr;
    GoalSource*              goals_       = nullptr;
    DataBuffer<NavCommand>*  cmd_buf_     = nullptr;
    NavCommandPublisher*     pub_         = nullptr;
    NavStatusPublisher*      status_pub_  = nullptr;
    bool                     drive_enabled_ = false;
    double                   arrival_hold_s_ = 1.0;

    // Arrival state machine (see resolve_state).
    double      arrival_hold_ = 0.0;      // accumulated time in phase Arrived (s)
    bool        holding_      = false;    // arrived + consumed -> report ARRIVED until a new goal
    std::string arrived_name_;            // name reported while holding_ (the goal we arrived at)

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
