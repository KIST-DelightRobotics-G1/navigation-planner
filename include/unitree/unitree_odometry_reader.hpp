#pragma once

#include "common/data_buffer.hpp"
#include "unitree/unitree_odometry.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace unitree::robot {
template <typename T> class ChannelSubscriber;
}
namespace nav_msgs::msg::dds_ {
class Odometry_;
}

namespace kist {

// The G1 locomotion controller's odometry. Published as the ROS2 topic
// /dog_odom (nav_msgs/Odometry, ~50Hz measured, BestEffort); over the
// DDS name mapping (rt/ prefix) it is reachable without ROS2. The
// onboard odom_node merely relays this topic, so we subscribe to the
// origin directly.
inline constexpr const char* kDefaultOdomTopic = "rt/dog_odom";

// Odometry samples from the robot's DDS relay (no ROS2) — the same
// subscribe pattern as UnitreePointCloudReader / gearsonic's
// UnitreeStateReader. Fixed-size payload, so the SDK→kist conversion is
// internal (gearsonic convert() style); there is no processor hook.
//
// The watchdog clears the buffer when samples stop arriving; consumers
// key off "has data" (empty-buffer principle).
class UnitreeOdometryReader {
public:
    static UnitreeOdometryReader& instance();

    bool start(int domain_id, const std::string& network_interface,
               const std::string& topic = kDefaultOdomTopic);
    void stop();

    // ── data buffer (read from any thread) ─────────────────────
    DataBuffer<UnitreeOdometry> odom_buf;

    // ── internal: DDS callback ──────────────────────────────────
    void on_odom_update(const void* message);

private:
    UnitreeOdometryReader() = default;

    void watchdog_loop();

    using OdomSub = unitree::robot::ChannelSubscriber<nav_msgs::msg::dds_::Odometry_>;
    std::unique_ptr<OdomSub> odom_sub_;

    std::thread       watchdog_thread_;
    std::atomic<bool> stop_watchdog_{false};
};

} // namespace kist
