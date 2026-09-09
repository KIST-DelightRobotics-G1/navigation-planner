#pragma once

// FAST-LIO2 wrapper. The heavy FAST-LIO / PCL / ikd-Tree state lives in the .cpp as
// file-scope globals (FAST-LIO's h_share_model is a free function the ESIKF calls by
// pointer, so it must reach the state globally) — hence exactly ONE LioWorker per
// process. This header exposes only Eigen + our types (PIMPL), so nothing FAST-LIO
// leaks to the rest of the tree.
//
//   process(frame, imu_window)  ->  T_odom_lidar
//
// The runtime drains the cloud + IMU queues, pairs a Mid-360 frame with the IMU
// samples spanning it, and calls process() per frame.

#include "unitree/unitree_pointcloud.hpp"
#include "unitree/unitree_pointcloud_imu.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <deque>
#include <memory>

namespace kist {

struct LioConfig {
    // Extrinsic: the LiDAR frame expressed in the IMU frame (FAST-LIO's
    // Lidar_T/R_wrt_IMU). Default = the standard Livox Mid-360 lidar<->imu extrinsic
    // (the built-in IMU sits ~4.4 cm below / 2.3 cm to the side of the LiDAR centre;
    // the two frames are axis-aligned). FAST-LIO also refines it online.
    Eigen::Vector3d ext_t = Eigen::Vector3d(-0.011, -0.02329, 0.04412);
    Eigen::Matrix3d ext_R = Eigen::Matrix3d::Identity();

    double blind            = 0.5;   // drop returns closer than this (m)
    int    point_filter_num = 3;     // keep every Nth raw point
    double filter_size_surf = 0.5;   // scan voxel downsample (m)
    double filter_size_map  = 0.5;   // ikd-Tree map voxel (m)
    double cube_len         = 200.0; // local map cube side (m)
    double det_range        = 100.0; // map-move trigger range (m)
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    int    max_iterations   = 4;     // ESIKF iterations
};

struct LioPose {                     // T_odom_lidar(t)
    int64_t            stamp_ns    = 0;
    Eigen::Vector3d    position    = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    linear_velocity = Eigen::Vector3d::Zero();  // odom frame
    bool               valid       = false;   // false while still initializing
};

class LioWorker {
public:
    explicit LioWorker(const LioConfig& cfg = {});
    ~LioWorker();
    LioWorker(const LioWorker&) = delete;
    LioWorker& operator=(const LioWorker&) = delete;

    // One synced step. `frame` = raw DDS Mid-360 cloud (needs per-point time);
    // `imu` = samples covering [frame start, frame end], ascending. Returns the
    // current lidar pose in odom.
    LioPose process(const UnitreePointCloud& frame, const std::deque<ImuSample>& imu);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace kist
