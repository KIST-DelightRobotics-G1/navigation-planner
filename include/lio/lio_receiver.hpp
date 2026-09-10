#pragma once

// LioReceiver — the planner's single consumer of the external LIO engine (docs/
// LIO_ENGINE.md). It owns two DDS subscriptions and decodes the engine's ROS2 output
// into plain structs, ROS-free (unitree_sdk2 / CycloneDDS, no rclcpp):
//
//   rt/Odometry_loc      (nav_msgs/Odometry)     -> LioOdometry -> odom_buf
//   rt/cloud_registered_1(sensor_msgs/PointCloud2)-> LioCloud    -> cloud_buf
//
// Threads: each subscription runs its handler on its own unitree_sdk2 receive thread
// (queuelen=1), so odom and cloud never block each other. No watchdog — a consumer
// keys off buffer age via GetDataWithTime().

#include "common/data_buffer.hpp"
#include "lio/lio_odometry.hpp"
#include "lio/lio_cloud.hpp"

#include <functional>
#include <memory>
#include <string>

namespace unitree::robot {
template <typename T> class ChannelSubscriber;
}
namespace nav_msgs::msg::dds_ {
class Odometry_;
}
namespace sensor_msgs::msg::dds_ {
class PointCloud2_;
}

namespace kist {

// DDS names (ROS2 topics get the "rt/" prefix over the DDS mapping).
inline constexpr const char* kDefaultLioOdomTopic  = "rt/Odometry_loc";
inline constexpr const char* kDefaultLioCloudTopic = "rt/cloud_registered_1";

class LioReceiver {
public:
    LioReceiver();          // out-of-line (unique_ptr to an incomplete ChannelSubscriber)
    ~LioReceiver();

    bool start(int domain_id, const std::string& network_interface = "",
               const std::string& odom_topic  = kDefaultLioOdomTopic,
               const std::string& cloud_topic = kDefaultLioCloudTopic);
    void stop();

    // ── outputs (read from any thread) ─────────────────────────
    DataBuffer<LioOdometry> odom_buf;    // latest T_odom_lidar + velocity
    DataBuffer<LioCloud>    cloud_buf;   // latest registered cloud (odom frame)

    // Optional per-frame hook: called on the LIO receive thread with each decoded
    // LioOdometry (before odom_buf is updated). The transform producer registers this
    // to run per LIO frame — reading the latest lowstate and composing T_odom_pelvis —
    // so no extra thread is needed. Set before start(); keep it cheap (it blocks Rx).
    using OdomHook = std::function<void(const LioOdometry&)>;
    void set_odom_hook(OdomHook hook) { odom_hook_ = std::move(hook); }

    LioReceiver(const LioReceiver&) = delete;
    LioReceiver& operator=(const LioReceiver&) = delete;

private:
    void on_odometry(const void* message);
    void on_cloud(const void* message);

    OdomHook odom_hook_;

    using OdomSub  = unitree::robot::ChannelSubscriber<nav_msgs::msg::dds_::Odometry_>;
    using CloudSub = unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>;
    std::unique_ptr<OdomSub>  odom_sub_;
    std::unique_ptr<CloudSub> cloud_sub_;
};

} // namespace kist
