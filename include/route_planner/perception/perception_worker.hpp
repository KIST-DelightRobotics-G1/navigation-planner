#pragma once

// PerceptionWorker — the perception thread: pull registered LIO scans + stamp-matched pose and
// drive the ObstacleMapper (pure logic), publishing grid + costmap snapshots to buffers. Owns
// its thread + loop (gearsonic-style self-contained worker); the map-building LOGIC stays in
// ObstacleMapper. Deps injected at start(); nothing here but wiring + rate.

#include "common/data_buffer.hpp"
#include "lio/lio_receiver.hpp"
#include "lio/lio_transform_producer.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"
#include "route_planner/perception/obstacle_grid_builder/obstacle_grid.hpp"
#include "route_planner/perception/obstacle_mapper.hpp"

#include <atomic>
#include <thread>

namespace kist {

class PerceptionWorker {
public:
    ~PerceptionWorker() { stop(); }

    void start(LioReceiver& rx, LioTransformProducer& prod,
               DataBuffer<ObstacleGrid>& grid_buf, DataBuffer<Costmap>& costmap_buf);
    void stop();

    const ObstacleGridConfig& gcfg() const { return mapper_.gcfg; }   // for viz (resolution/band)

private:
    void run();

    ObstacleMapper            mapper_;
    LioReceiver*              rx_        = nullptr;
    LioTransformProducer*     prod_      = nullptr;
    DataBuffer<ObstacleGrid>* grid_buf_  = nullptr;
    DataBuffer<Costmap>*      costmap_buf_ = nullptr;

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
