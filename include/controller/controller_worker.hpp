#pragma once

// ControllerWorker — the control thread (~20 Hz): gather the freshest inputs (path, pose,
// costmap, goal), drive LocalController (pure logic), then EITHER publish the command as a Twist
// (drive_enabled -> robot moves) OR just preview it. Owns its thread + loop; the control DECISION
// stays in LocalController. Stale inputs (>500 ms) are nulled so the controller stops safely.

#include "common/data_buffer.hpp"
#include "controller/local_controller.hpp"
#include "controller/nav_command.hpp"
#include "controller/nav_command_publisher.hpp"
#include "controller/subtask_state_publisher.hpp"
#include "goal_generation/goal.hpp"
#include "lio/lio_transform_producer.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"
#include "route_planner/planner/astar_planner/path.hpp"
#include "system/goal_source.hpp"

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>

namespace kist {

class ControllerWorker {
public:
    ~ControllerWorker() { stop(); }

    // `status_pub` publishes SubtaskState on rt/cortex/nav/state (10 Hz). `arrival_hold_s` is the
    // arrival debounce (hold at goal this long -> DONE); `nopath_hold_s` is the no-path debounce
    // (a transient no-path is reported RUNNING; only a sustained one this long -> FAILED "no path").
    void start(DataBuffer<Path>& path_buf, LioTransformProducer& prod, DataBuffer<Costmap>& costmap_buf,
               GoalSource& goals, DataBuffer<NavCommand>& cmd_buf, NavCommandPublisher& pub,
               SubtaskStatePublisher& status_pub, bool drive_enabled, const FollowConfig& fc,
               double arrival_hold_s = 1.0, double nopath_hold_s = 4.0);
    void stop();

private:
    void run();

    // What to report on rt/cortex/nav/state this cycle.
    struct SubtaskReport {
        SubtaskStatus status  = SubtaskStatus::Idle;
        float         progress = 0.0f;   // 0..1
        std::string   note;              // "", "cancelled", "no path", "unsupported: ..."
        std::string   plan_id;           // "" when idle
        uint16_t      index  = 0;
        std::string   action;            // "" when idle
    };
    // Fold the follower phase + goal (+ robot pose for progress) into a SubtaskReport, debouncing
    // arrival (must hold arrival_hold_s -> DONE) and consuming the goal on the confirmed edge.
    SubtaskReport step_status(const std::optional<Goal>& goal, const RobotTransforms* rt,
                              FollowPhase phase, double dt);

    LocalController          ctrl_;
    DataBuffer<Path>*        path_buf_    = nullptr;
    LioTransformProducer*    prod_        = nullptr;
    DataBuffer<Costmap>*     costmap_buf_ = nullptr;
    GoalSource*              goals_       = nullptr;
    DataBuffer<NavCommand>*  cmd_buf_     = nullptr;
    NavCommandPublisher*     pub_         = nullptr;
    SubtaskStatePublisher*   status_pub_  = nullptr;
    bool                     drive_enabled_ = false;
    double                   arrival_hold_s_ = 1.0;
    double                   nopath_hold_s_  = 4.0;

    // Subtask / arrival state machine (see step_status).
    double      arrival_hold_ = 0.0;      // accumulated time in phase Arrived (s)
    double      nopath_hold_  = 0.0;      // accumulated time in phase NoPath (s) -> FAILED when sustained
    bool        holding_      = false;    // arrived + consumed -> report DONE until a new subtask
    std::string cur_plan_;                // subtask being tracked (for change detection / progress)
    uint16_t    cur_index_ = 0;
    bool        have_cur_   = false;
    float       initial_dist_ = 0.0f;     // robot->goal distance captured at subtask start (progress)
    std::string arr_plan_, arr_action_;   // subtask reported while holding_ (the one we reached)
    uint16_t    arr_index_ = 0;

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
