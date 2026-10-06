#pragma once

// PerceptionWorker — the perception thread: pull registered LIO scans + stamp-matched pose and
// drive the ObstacleMapper (pure logic), publishing grid + costmap snapshots to buffers. Owns
// its thread + loop (gearsonic-style self-contained worker); the map-building LOGIC stays in
// ObstacleMapper. Deps injected at start(); nothing here but wiring + rate.

#include "common/data_buffer.hpp"
#include "lio/lio_receiver.hpp"
#include "lio/lio_transform_producer.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"
#include "route_planner/perception/ground_leveler.hpp"
#include "route_planner/perception/obstacle_grid_builder/obstacle_grid.hpp"
#include "route_planner/perception/obstacle_mapper.hpp"

#include <atomic>
#include <thread>

namespace kist {

class PerceptionWorker {
public:
    ~PerceptionWorker() { stop(); }

    void start(LioReceiver& rx, LioTransformProducer& prod,
               DataBuffer<ObstacleGrid>& grid_buf, DataBuffer<Costmap>& costmap_buf,
               DataBuffer<Transform>& leveled_buf);   // T_leveled_odom out (for viz + downstream)
    void stop();

    const ObstacleGridConfig& gcfg() const { return mapper_.gcfg; }   // for viz (resolution/band)

    // Fresh LIO session (odom origin redefined): drop the accumulated grid + re-estimate the floor
    // tilt. Clears leveled_locked_ SYNCHRONOUSLY so a coordinator polling it never sees the stale
    // previous lock; the grid is rebuilt (and leveled_locked_ set) once the leveler re-locks.
    void request_reset() { leveled_locked_.store(false); reset_req_.store(true); }

    // Has the ground leveler locked? The obstacle grid is built ONLY after this (gravity-aligned
    // frame), so this gates "safe to drive" — a goal/path over a tilted pre-lock grid is unsafe.
    bool leveled_locked() const { return leveled_locked_.load(); }

private:
    void run();

    ObstacleMapper            mapper_;
    GroundLeveler             leveler_;   // estimates T_leveled_odom (diagnostic-only for now)
    LioReceiver*              rx_        = nullptr;
    LioTransformProducer*     prod_      = nullptr;
    DataBuffer<ObstacleGrid>* grid_buf_  = nullptr;
    DataBuffer<Costmap>*      costmap_buf_ = nullptr;
    DataBuffer<Transform>*    leveled_buf_ = nullptr;   // T_leveled_odom (world-fixed, ~static)

    std::thread       thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> reset_req_{false};        // fresh LIO: clear grid + re-lock the ground leveler
    std::atomic<bool> leveled_locked_{false};   // ground leveler locked -> grid is gravity-aligned
};

} // namespace kist
