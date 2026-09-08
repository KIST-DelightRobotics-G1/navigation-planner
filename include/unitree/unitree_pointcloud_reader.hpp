#pragma once

#include "common/data_buffer.hpp"
#include "common/record_queue.hpp"
#include "unitree/unitree_pointcloud.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
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

// The G1's Livox Mid-360 cloud. The Unitree internal driver publishes it
// as the ROS2 topic /utlidar/cloud_livox_mid360 (Reliable, keep-last 1);
// over the DDS name mapping (rt/ prefix) it is reachable without ROS2.
// frame_id arrives as "livox_frame" — the onboard lidar_node merely
// relays this topic (rewriting frame_id to "utlidar_lidar"), so we
// subscribe to the origin directly.
inline constexpr const char* kDefaultCloudTopic = "rt/utlidar/cloud_livox_mid360";

// LiDAR frames from the robot's DDS relay (no ROS2, no Livox SDK) —
// the same subscribe pattern as gearsonic's UnitreeStateReader.
//
// The watchdog clears the buffer when frames stop arriving; consumers
// key off "has data" (empty-buffer principle).
class UnitreePointCloudReader {
public:
    static UnitreePointCloudReader& instance();

    bool start(int domain_id, const std::string& network_interface,
               const std::string& topic = kDefaultCloudTopic,
               std::size_t queue_capacity = 16);   // frames buffered for the LIO worker
    void stop();

    // Optional post-decode hook, run inside the DDS receive callback
    // before the frame is published to cloud_buf. Keeps the reader a
    // pure transport layer: what to run (e.g. a pointcloud processor)
    // is decided at the assembly point. Must stay cheap (a few ms) —
    // it blocks the receive thread. Set before start(); not guarded
    // against mutation while frames are flowing.
    using ProcessFn = std::function<void(UnitreePointCloud&)>;
    void set_processor(ProcessFn fn);

    // ── outputs (read from any thread) ─────────────────────────
    // cloud_queue: EVERY frame, for the LIO worker to drain (pop_all) time-synced
    // with the IMU queue. cloud_buf: latest-only snapshot (quick peeks + the
    // staleness watchdog). Prefer the queue for LIO; a dropped frame (queue full =
    // consumer stalled) bumps `dropped`.
    RecordQueue<UnitreePointCloud> cloud_queue;
    DataBuffer<UnitreePointCloud>  cloud_buf;
    std::atomic<uint64_t>          dropped{0};

    // ── internal: DDS callback ──────────────────────────────────
    void on_cloud_update(const void* message);

private:
    UnitreePointCloudReader() = default;

    void watchdog_loop();

    using CloudSub = unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>;
    std::unique_ptr<CloudSub> cloud_sub_;

    std::thread       watchdog_thread_;
    std::atomic<bool> stop_watchdog_{false};

    ProcessFn process_;
};

} // namespace kist
