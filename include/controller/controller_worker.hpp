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

    // What to report on rt/cortex/nav/state this tick.
    struct SubtaskReport {
        SubtaskStatus status  = SubtaskStatus::Idle;
        float         progress = 0.0f;   // 0..1
        std::string   detail;            // "", "cancelled", "no path", "unsupported: ..."
        std::string   plan_id;           // "" when idle
        uint16_t      index  = 0;
        std::string   action;            // "" when idle
    };
    // The status FSM, called once per publish (10 Hz). Debounces arrival (-> DONE) and no-path
    // (-> FAILED), then emits the terminal verdict for exactly 3 publishes before returning to IDLE
    // (cortex contract). Consumes the goal when a subtask terminates. `dt` = s since the last call.
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
    double      arrival_hold_ = 0.0;      // accumulated time in phase Arrived (s) -> DONE when held
    double      nopath_hold_  = 0.0;      // accumulated time in phase NoPath  (s) -> FAILED when held
    std::string cur_plan_;                // subtask being tracked (for change detection / progress)
    uint16_t    cur_index_ = 0;
    bool        have_cur_   = false;
    float       initial_dist_ = 0.0f;     // robot->goal distance captured at subtask start (progress)

    // Terminal burst: DONE/FAILED is emitted for exactly 3 publishes, then IDLE.
    bool          term_active_ = false;
    int           term_count_  = 0;
    SubtaskStatus term_status_ = SubtaskStatus::Idle;
    std::string   term_plan_, term_action_, term_detail_;
    uint16_t      term_index_    = 0;
    float         term_progress_ = 0.0f;
    // The subtask whose terminal burst has completed — suppresses re-reporting a lingering verdict.
    std::string   closed_plan_;
    uint16_t      closed_index_ = 0;
    bool          have_closed_  = false;

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
