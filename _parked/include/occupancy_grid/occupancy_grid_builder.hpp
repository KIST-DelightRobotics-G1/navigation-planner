#pragma once

// Worker thread that fuses the LiDAR cloud + the labeled camera cloud into a
// probabilistic semantic occupancy grid, and publishes it on its own buffer.
// Inputs are two DataBuffers (KF/reader style); the update math is the free
// functions in occupancy_grid.hpp. The grid ACCUMULATES across frames (log-odds
// belief), so the worker owns the persistent grid and publishes a snapshot each
// cycle. A consumer only start()/stop()s it and reads result().

#include "common/data_buffer.hpp"
#include "occupancy_grid/occupancy_grid.hpp"      // OccupancyGrid, GridConfig
#include "occupancy_grid/object_cluster.hpp"      // ObjectList, extract_clusters
#include "scan_match/scan_matcher.hpp"            // ScanMatchConfig, scan_match
#include "labeled_cloud/labeled_cloud.hpp"        // LabeledCloud
#include "unitree/unitree_pointcloud.hpp"         // UnitreePointCloud
#include "unitree/unitree_odometry.hpp"           // UnitreeOdometry (base orientation -> leveling)
#include "unitree/g1_lowstate_reader.hpp"         // WaistJoints
#include "kinematics/g1_kinematics.hpp"          // G1Kinematics::pelvisToTorso (torso bob) -> Transform
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
               DataBuffer<UnitreeOdometry>&   odom_src,   // base orientation -> tilt leveling
               DataBuffer<WaistJoints>&       waist_src,  // waist joints -> FK torso-bob leveling
               const GridConfig& cfg,
               const ScanMatchConfig& sm_cfg = {});       // LiDAR scan-match pose refinement
    void stop();

    bool running() const { return running_; }

    DataBuffer<OccupancyGrid>& result()  { return grid_buf_; }
    DataBuffer<ObjectList>&    objects() { return objects_buf_; }  // clusters, same cadence

    // Live toggle for the scan-match A/B test (benign bool race with the worker).
    void set_scan_match(bool on) { sm_cfg_.enabled = on; }
    bool scan_match_on() const   { return sm_cfg_.enabled; }
    uint64_t frames_processed() const { return processed_.load(std::memory_order_relaxed); }

private:
    void run();

    DataBuffer<UnitreePointCloud>* lidar_src_ = nullptr;
    DataBuffer<LabeledCloud>*      cloud_src_ = nullptr;
    DataBuffer<CalibratedPose>*    pose_src_  = nullptr;
    DataBuffer<UnitreeOdometry>*   odom_src_  = nullptr;
    DataBuffer<WaistJoints>*       waist_src_ = nullptr;
    Transform waist_ref_inv_;          // inverse of the reference (standing) waist FK
    bool  waist_ref_set_ = false;      // captured on the first waist sample
    DataBuffer<OccupancyGrid>      grid_buf_;
    DataBuffer<ObjectList>         objects_buf_;

    GridConfig      cfg_;
    ScanMatchConfig sm_cfg_;
    OccupancyGrid   grid_;        // persistent, accumulates across frames
    std::vector<float> scan2d_;   // reused robot-frame 2D scan for matching

    uint64_t              dbg_ = 0;    // throttle counter for the tilt diagnostic log
    std::thread           thread_;
    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> processed_{0};
};

} // namespace kist
