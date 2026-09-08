#pragma once

// Planner worker thread: OccupancyGrid -> Costmap (EDT) -> A* path.
//
// costmap and A* are a strict sequential pipeline (A* always consumes the
// costmap just built), so they share ONE thread — splitting them would only add
// a buffer + a wait, no concurrency. The worker reads the grid + object buffers
// (the OccupancyGridBuilder's outputs), rebuilds the cost field each new grid,
// plans toward an externally-set goal, and publishes a PlanResult on its own
// buffer. A consumer set_goal()s it, start()/stop()s it, and reads result().

#include "common/data_buffer.hpp"
#include "planner/costmap/costmap_builder.hpp"
#include "planner/astar/astar_planner.hpp"
#include "occupancy_grid/occupancy_grid.hpp"   // OccupancyGrid, GridConfig
#include "occupancy_grid/object_cluster.hpp"   // ObjectList (dynamic_mask)

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace kist {

struct PlannerConfig {
    GridConfig    grid;      // occ_threshold + is_dynamic, needed by the costmap build
    CostmapConfig costmap;   // inflation radii / cost shape
    AStarConfig   astar;     // search weights + distance trust

    // Replan policy. The costmap is rebuilt every grid frame (cheap), but A* runs
    // only on a trigger — otherwise the path jitters between near-equal-cost
    // routes. The committed path is kept while it stays valid. The cheap peak-cost
    // check runs EVERY cycle; A* reruns only once the path has DEGRADED (its peak
    // cell cost reached the threshold — grazing an obstacle, before it's fully
    // blocked). The replan is optimal for the current costmap, so it never comes
    // out worse. (A* reruns are internally capped to 10Hz so a stuck, still-degraded
    // path can't hammer the search — see kBackstopMinInterval_s in planner.cpp.)
    float replan_deviation_m    = 0.5f;   // replan if the robot strays this far from the path
    float replan_cost_threshold = 150.0f; // backstop reroutes when the path's peak cell cost reaches this (0..lethal); higher = more tolerant
};

// Worker output: the cost field the plan was made on, plus the plan itself.
// Both live here (same place as the worker) so a consumer gets a consistent
// (costmap, path) pair from one buffer read.
struct PlanResult {
    int64_t stamp_ns = 0;
    Costmap costmap;         // cost field (people excluded when ignore_dynamic)
    Path    path;            // robot -> goal; empty if no goal / unreachable
    bool    has_goal = false;

    // Objects from the ObjectList whose footprint (bbox grown by the robot
    // radius) the committed path crosses. Non-person obstacles are already
    // avoided by A*, so this is dominated by people standing on the path — the
    // trigger for the stop-and-ask behaviour (class_id 0 == person).
    std::vector<DetectedObject> blocking;
};

class Planner {
public:
    Planner() = default;
    ~Planner() { stop(); }

    Planner(const Planner&) = delete;
    Planner& operator=(const Planner&) = delete;

    // Start off the grid + object buffers (the OccupancyGridBuilder's outputs).
    // False if already running.
    bool start(DataBuffer<OccupancyGrid>& grid_src,
               DataBuffer<ObjectList>&    objects_src,
               const PlannerConfig&       cfg);
    void stop();

    bool running() const { return running_; }

    // Goal in the world/map frame (the grid's frame). Thread-safe; the next cycle
    // plans toward it. clear_goal() drops it (the worker keeps building the
    // costmap but publishes an empty path).
    void set_goal(float x, float y);
    void clear_goal();

    DataBuffer<PlanResult>& result() { return result_; }
    uint64_t frames_processed() const { return processed_.load(std::memory_order_relaxed); }

private:
    void run();

    DataBuffer<OccupancyGrid>* grid_src_    = nullptr;
    DataBuffer<ObjectList>*    objects_src_ = nullptr;
    PlannerConfig              cfg_;

    CostmapBuilder cm_builder_;   // caches its distance->cost LUT
    AStarPlanner   astar_;        // reconstructed from cfg_.astar in start()

    std::mutex goal_mtx_;
    bool     has_goal_ = false;
    float    goal_x_ = 0.0f, goal_y_ = 0.0f;
    uint64_t goal_epoch_ = 0;   // bumped on each set_goal -> lets run() detect a new goal

    DataBuffer<PlanResult> result_;
    std::thread           thread_;
    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> processed_{0};
};

} // namespace kist
