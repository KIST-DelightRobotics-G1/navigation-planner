#pragma once

#include "common/data_buffer.hpp"
#include "common/pointcloud_frame.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace unitree::robot {
template <typename T> class ChannelSubscriber;
}
namespace sensor_msgs::msg::dds_ {
class PointCloud2_;
}

namespace kist {

// LiDAR frames from the robot's DDS relay (no ROS2, no sensor SDK) —
// the same subscribe pattern as gearsonic's UnitreeStateReader.
//
// The watchdog clears the buffer when frames stop arriving; consumers
// key off "has data" (empty-buffer principle).
class LidarReader {
public:
    static LidarReader& instance();

    // topic: the robot's point-cloud relay. Default is the Unitree
    // utlidar deskewed cloud — verify the exact name on the robot.
    bool start(int domain_id, const std::string& network_interface,
               const std::string& topic = "rt/utlidar/cloud_deskewed");
    void stop();

    // ── data buffer (read from any thread) ─────────────────────
    DataBuffer<PointCloudXYZFrame> cloud_buf;

    // ── internal: DDS callback ──────────────────────────────────
    void on_cloud_update(const void* message);

private:
    LidarReader() = default;

    void watchdog_loop();

    using CloudSub = unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>;
    std::unique_ptr<CloudSub> cloud_sub_;

    std::thread       watchdog_thread_;
    std::atomic<bool> stop_watchdog_{false};
};

} // namespace kist
