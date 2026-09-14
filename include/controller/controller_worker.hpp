#pragma once

// ControllerWorker — the control thread (~20 Hz): gather the freshest inputs (path, pose,
// costmap, goal), drive LocalController (pure logic), then EITHER publish the command as a Twist
// (drive_enabled -> robot moves) OR just preview it. Owns its thread + loop; the control DECISION
// stays in LocalController. Stale inputs (>500 ms) are nulled so the controller stops safely.

#include "common/data_buffer.hpp"
#include "controller/local_controller.hpp"
#include "controller/nav_command.hpp"
#include "controller/nav_command_publisher.hpp"
#include "lio/lio_transform_producer.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"
#include "route_planner/planner/astar_planner/path.hpp"
#include "system/goal_source.hpp"

#include <atomic>
#include <thread>

namespace kist {

class ControllerWorker {
public:
    ~ControllerWorker() { stop(); }

    void start(DataBuffer<Path>& path_buf, LioTransformProducer& prod, DataBuffer<Costmap>& costmap_buf,
               GoalSource& goals, DataBuffer<NavCommand>& cmd_buf, NavCommandPublisher& pub,
               bool drive_enabled, const FollowConfig& fc);
    void stop();

private:
    void run();

    LocalController          ctrl_;
    DataBuffer<Path>*        path_buf_    = nullptr;
    LioTransformProducer*    prod_        = nullptr;
    DataBuffer<Costmap>*     costmap_buf_ = nullptr;
    GoalSource*              goals_       = nullptr;
    DataBuffer<NavCommand>*  cmd_buf_     = nullptr;
    NavCommandPublisher*     pub_         = nullptr;
    bool                     drive_enabled_ = false;

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
