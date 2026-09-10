#pragma once

// LioTreeProducer — feeds the LIO engine's pose into the transform tree.
//
//   static : lidar_imu -> lidar   (Mid-360 datasheet ext, set once at construction)
//   dynamic: odom      -> lidar_imu (appended from each LioOdometry, with its stamp)
//
// After update(), tree.lookupTransform(Odom, Lidar, t) composes both edges into the true
// T_odom_lidar(t) — the LIO pose (IMU body) plus the fixed imu->lidar offset.
//
// The transform tree is NOT thread-safe: call update() (write) and lookupTransform()
// (read) from the SAME thread, or guard the tree with a mutex if the producer and the
// consumers live on different threads (see lio_receiver.hpp for the DDS-thread model).

#include "frames/frame_ids.hpp"
#include "lio/lio_odometry.hpp"
#include "transforms/transform.hpp"
#include "transforms/transform_tree.hpp"

#include <Eigen/Core>

namespace kist {

class LioTreeProducer {
public:
    // Sets the fixed lidar_imu -> lidar edge (the LIO engine already handles the
    // upside-down mount; this is only the internal imu<->lidar offset, axes aligned).
    explicit LioTreeProducer(TransformTree& tree) : tree_(tree) {
        Transform T_imu_lidar;                                   // rotation = identity
        T_imu_lidar.translation = Eigen::Vector3d(-0.011, -0.02329, 0.04412);  // datasheet
        tree_.setStaticTransform(FrameId::LidarImu, FrameId::Lidar, T_imu_lidar);
    }

    // Append the latest LIO pose as the odom -> lidar_imu dynamic edge.
    void update(const LioOdometry& od) {
        tree_.updateTransform(FrameId::Odom, FrameId::LidarImu,
                              od.pose.T_parent_child, od.pose.stamp_ns);
    }

private:
    TransformTree& tree_;
};

} // namespace kist
