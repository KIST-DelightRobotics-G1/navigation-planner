#pragma once

// PlannerWorker — the planner thread: read the latest costmap + active goal and drive the
// RoutePlanner (pure logic), publishing a Path to a buffer at the replan rate. Owns its thread
// + loop; the routing LOGIC stays in RoutePlanner. Deps injected at start().

#include "common/data_buffer.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"
#include "route_planner/planner/astar_planner/path.hpp"
#include "route_planner/planner/route_planner.hpp"
#include "system/goal_source.hpp"

#include <atomic>
#include <thread>

namespace kist {

class PlannerWorker {
public:
    ~PlannerWorker() { stop(); }

    void start(DataBuffer<Costmap>& costmap_buf, GoalSource& goals, DataBuffer<Path>& path_buf);
    void stop();

private:
    void run();

    RoutePlanner         planner_;
    DataBuffer<Costmap>* costmap_buf_ = nullptr;
    GoalSource*          goals_       = nullptr;
    DataBuffer<Path>*    path_buf_    = nullptr;

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
