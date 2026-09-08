// Mid-360 IMU discovery probe. The LIO / deskew path needs the LiDAR's built-in
// IMU, which the utlidar point-cloud relay strips. sensor_msgs/Imu IS a bundled
// DDS type, so if Unitree relays the Livox IMU on some topic we can subscribe with
// the same pattern as the cloud reader — this probe finds that topic and confirms
// data is flowing.
//   ./test_lidar_imu_probe [config.yaml] [topic]
// Try candidate topics without rebuilding, e.g.:
//   ./test_lidar_imu_probe config/config.yaml rt/utlidar/imu
//   ./test_lidar_imu_probe config/config.yaml rt/utlidar/imu_livox_mid360
//   ./test_lidar_imu_probe config/config.yaml rt/livox/imu

#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/Imu_.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};
static std::atomic<long> g_count{0};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string cfg   = (argc >= 2) ? argv[1] : "config/config.yaml";
    const std::string topic = (argc >= 3) ? argv[2] : "rt/utlidar/imu";

    kist::Config::instance().load(cfg);
    const auto& root = kist::Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;

    std::signal(SIGINT, [](int) { g_stop = true; });

    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain, "");
        static unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::Imu_> sub(topic);
        sub.InitChannel([](const void* m) {
            const auto& imu = *static_cast<const sensor_msgs::msg::dds_::Imu_*>(m);
            const long n = ++g_count;
            const auto& g = imu.angular_velocity();
            const auto& a = imu.linear_acceleration();
            const int64_t t = int64_t(imu.header().stamp().sec()) * 1000000000LL +
                              imu.header().stamp().nanosec();
            if (n <= 5 || n % 200 == 0)
                std::printf("[imu #%ld] t=%lld frame=%s gyro=(% .4f % .4f % .4f) "
                            "accel=(% .3f % .3f % .3f)\n",
                            n, static_cast<long long>(t), imu.header().frame_id().c_str(),
                            g.x(), g.y(), g.z(), a.x(), a.y(), a.z());
        }, 1);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "DDS init failed: %s\n", e.what());
        return 1;
    }

    std::printf("listening for sensor_msgs/Imu on \"%s\" (domain %d). Ctrl+C to quit.\n",
                topic.c_str(), domain);
    std::printf("no data? retry another topic, e.g.:\n"
                "  ./test_lidar_imu_probe %s rt/utlidar/imu_livox_mid360\n"
                "  ./test_lidar_imu_probe %s rt/livox/imu\n"
                "  ./test_lidar_imu_probe %s rt/utlidar/lidar/imu\n",
                cfg.c_str(), cfg.c_str(), cfg.c_str());

    int waited = 0;
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (g_count.load() == 0 && (++waited % 4 == 0))
            std::printf("...still nothing on \"%s\" (%ds)\n", topic.c_str(), waited / 2);
    }
    std::printf("total IMU messages on \"%s\": %ld\n", topic.c_str(), g_count.load());
    return 0;
}
