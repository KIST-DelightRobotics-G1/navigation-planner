#include "unitree/unitree_pointcloud_imu_reader.hpp"

#include <unitree/idl/ros2/Imu_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <exception>
#include <iostream>

namespace kist {

namespace {
// The Livox built-in IMU reports linear acceleration in g; LIO / ImuSample want
// m/s^2 (standard gravity).
constexpr double kGravity = 9.80665;
}  // namespace

UnitreePointCloudImuReader& UnitreePointCloudImuReader::instance() {
    static UnitreePointCloudImuReader inst;
    return inst;
}

bool UnitreePointCloudImuReader::start(int domain_id, const std::string& network_interface,
                                       const std::string& topic, std::size_t queue_capacity) {
    imu_queue.open(queue_capacity);
    try {
        // No-op after the first Init in the same process.
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        imu_sub_.reset(new ImuSub(topic));
        imu_sub_->InitChannel([this](const void* msg) { on_imu_update(msg); }, 50);
    } catch (const std::exception& e) {
        std::cerr << "[UnitreePointCloudImuReader] DDS init failed on interface \""
                  << network_interface << "\": " << e.what()
                  << "\n  Check the robot LAN cable.\n";
        return false;
    }
    std::cout << "[UnitreePointCloudImuReader] started on domain=" << domain_id
              << " topic=" << topic << "\n";
    return true;
}

void UnitreePointCloudImuReader::stop() {
    imu_queue.close();     // wake/drain the consumer
    imu_sub_.reset();
}

void UnitreePointCloudImuReader::on_imu_update(const void* message) {
    const auto& msg = *static_cast<const sensor_msgs::msg::dds_::Imu_*>(message);

    ImuSample s;
    s.stamp_ns = int64_t(msg.header().stamp().sec()) * 1000000000LL +
                 msg.header().stamp().nanosec();
    const auto& g = msg.angular_velocity();
    const auto& a = msg.linear_acceleration();
    s.gyro  = Eigen::Vector3d(g.x(), g.y(), g.z());                 // rad/s
    s.accel = Eigen::Vector3d(a.x(), a.y(), a.z()) * kGravity;      // g -> m/s^2

    if (!imu_queue.push(std::move(s)))
        dropped.fetch_add(1, std::memory_order_relaxed);
}

} // namespace kist
