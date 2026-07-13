#include "unitree/unitree_pointcloud_reader.hpp"
#include "unitree/pointcloud2_decode.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <iostream>

namespace kist {

UnitreePointcloudReader& UnitreePointcloudReader::instance() {
    static UnitreePointcloudReader inst;
    return inst;
}

bool UnitreePointcloudReader::start(int domain_id, const std::string& network_interface,
                        const std::string& topic) {
    try {
        // Safe when the embedding process already initialized the factory
        // (Init is a no-op after the first call in the same process).
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

        cloud_sub_.reset(new CloudSub(topic));
        cloud_sub_->InitChannel(
            [this](const void* msg) { on_cloud_update(msg); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[UnitreePointcloudReader] DDS init failed on interface \""
                  << network_interface << "\": " << e.what()
                  << "\n  Check the robot LAN cable and the topic name (\""
                  << topic << "\").\n";
        return false;
    }

    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&UnitreePointcloudReader::watchdog_loop, this);
    std::cout << "[UnitreePointcloudReader] started on domain=" << domain_id
              << " interface=" << network_interface
              << " topic=" << topic << "\n";
    return true;
}

void UnitreePointcloudReader::stop() {
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    cloud_sub_.reset();
}

void UnitreePointcloudReader::on_cloud_update(const void* message) {
    const auto& msg = *static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(message);

    PointCloudXYZFrame frame;
    if (!decode_pointcloud2(msg, frame)) {
        std::cerr << "[UnitreePointcloudReader] cloud without FLOAT32 x/y/z fields — dropped\n";
        return;
    }
    cloud_buf.SetData(std::move(frame));
}

// LiDAR streams at ~10Hz; 500ms (5 frames) of silence clears the buffer
// so downstream mapping stops extending a stale world.
void UnitreePointcloudReader::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 500.0;

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(50ms);

        auto cloud = cloud_buf.GetDataWithTime();
        if (cloud.HasData() && cloud.GetAgeMs() > stale_ms) {
            std::cerr << "[UnitreePointcloudReader] cloud stale — cleared\n";
            cloud_buf.Clear();
        }
    }
}

} // namespace kist
