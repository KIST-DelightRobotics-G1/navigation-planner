#pragma once

// The Mid-360's BUILT-IN IMU (ICM40609), relayed by the Unitree utlidar node as
// the ROS2 topic /utlidar/imu_livox_mid360 → rt/ over DDS, ~200 Hz. Named after
// the pointcloud/LiDAR sensor so it is never confused with the G1 BODY IMU
// (lowstate) — this is the IMU rigidly on the LiDAR, what LIO fuses.
//
// A SEPARATE reader from UnitreePointCloudReader (its own subscriber/callback
// path) on purpose: the IMU is ~20x the cloud rate, so it must not wait behind
// cloud decoding. Samples land in a bounded hand-off queue (RecordQueue) for the
// LIO worker to drain — a latest-only DataBuffer would drop most of a 200 Hz
// stream. Producer = the DDS callback; consumer = the LIO worker.

#include "common/record_queue.hpp"
#include "unitree/unitree_pointcloud_imu.hpp"     // ImuSample

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace unitree::robot {
template <typename T> class ChannelSubscriber;
}
namespace sensor_msgs::msg::dds_ {
class Imu_;
}

namespace kist {

inline constexpr const char* kDefaultLidarImuTopic = "rt/utlidar/imu_livox_mid360";

class UnitreePointCloudImuReader {
public:
    static UnitreePointCloudImuReader& instance();

    bool start(int domain_id, const std::string& network_interface,
               const std::string& topic = kDefaultLidarImuTopic,
               std::size_t queue_capacity = 4000);   // ~20 s at 200 Hz
    void stop();

    // IMU hand-off queue: producer = DDS callback, consumer = LIO worker.
    // Drain with imu_queue.pop_all(batch); accel is already in m/s^2.
    RecordQueue<ImuSample> imu_queue;

    // Samples dropped because the queue was full (i.e. the consumer stalled).
    std::atomic<uint64_t> dropped{0};

    void on_imu_update(const void* message);   // internal: DDS receive callback

private:
    UnitreePointCloudImuReader() = default;

    using ImuSub = unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::Imu_>;
    std::unique_ptr<ImuSub> imu_sub_;
};

} // namespace kist
