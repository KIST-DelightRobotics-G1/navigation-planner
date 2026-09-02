#pragma once

// Worker thread that fuses the LiDAR cloud + the labeled camera cloud into a
// probabilistic semantic occupancy grid, and publishes it on its own buffer.
// Inputs are two DataBuffers (KF/reader style); the update math is the free
// functions in occupancy_grid.hpp. The grid ACCUMULATES across frames (log-odds
// belief), so the worker owns the persistent grid and publishes a snapshot each
// cycle. A consumer only start()/stop()s it and reads result().

#include "common/data_buffer.hpp"
#include "occupancy_grid/occupancy_grid.hpp"      // OccupancyGrid, GridConfig
#include "labeled_cloud/labeled_cloud.hpp"        // LabeledCloud
#include "unitree/unitree_pointcloud.hpp"         // UnitreePointCloud
#include "kalman_filter/calibrated_pose.hpp"      // CalibratedPose

#include <atomic>
#include <cstdint>
#include <thread>

namespace kist {

class OccupancyGridBuilder {
public:
    OccupancyGridBuilder() = default;
    ~OccupancyGridBuilder() { stop(); }

    OccupancyGridBuilder(const OccupancyGridBuilder&) = delete;
    OccupancyGridBuilder& operator=(const OccupancyGridBuilder&) = delete;

    // Start the worker off the LiDAR + labeled-cloud + pose buffers. When a
    // trusted pose is present the grid is world-anchored and accumulates; when
    // pose is absent (calibrating / gated) it falls back to a robot-centered
    // memoryless snapshot so downstream is never left blind. False if running.
    bool start(DataBuffer<UnitreePointCloud>& lidar_src,
               DataBuffer<LabeledCloud>&      cloud_src,
               DataBuffer<CalibratedPose>&    pose_src,
               const GridConfig& cfg);
    void stop();

    bool running() const { return running_; }

    DataBuffer<OccupancyGrid>& result() { return grid_buf_; }
    uint64_t frames_processed() const { return processed_.load(std::memory_order_relaxed); }

private:
    void run();

    DataBuffer<UnitreePointCloud>* lidar_src_ = nullptr;
    DataBuffer<LabeledCloud>*      cloud_src_ = nullptr;
    DataBuffer<CalibratedPose>*    pose_src_  = nullptr;
    DataBuffer<OccupancyGrid>      grid_buf_;

    GridConfig    cfg_;
    OccupancyGrid grid_;          // persistent, accumulates across frames

    std::thread           thread_;
    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> processed_{0};
};

} // namespace kist
