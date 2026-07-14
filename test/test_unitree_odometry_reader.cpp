// Live odometry stream check (robot LAN needed):
//   ./test_unitree_odometry_reader [config_path]
// Interface/domain come from config/config.yaml. Prints sample age,
// position, yaw, and forward velocity once per second. Ctrl-C to stop.
//
// If the pointcloud stream works but this prints "no odometry", suspect
// a QoS mismatch first: /dog_odom is published BestEffort, so the DDS
// reader must not demand Reliable.

#include "common/config.hpp"
#include "unitree/unitree_odometry_reader.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);

    const auto unitree_cfg = Config::instance().root()["unitree"];
    const auto domain_id   = unitree_cfg["domain_id"].as<int>();
    const auto interface   = unitree_cfg["network_interface"].as<std::string>();

    std::signal(SIGINT, [](int) { g_stop = true; });

    auto& reader = UnitreeOdometryReader::instance();
    if (!reader.start(domain_id, interface))
        return 1;

    // Receive rate is measured by polling for stamp changes at 1ms —
    // reliable up to a few hundred Hz, no reader-side counter needed.
    int64_t last_stamp = 0;
    int     frames = 0;
    auto    window_start = std::chrono::steady_clock::now();

    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        auto odom = reader.odom_buf.GetDataWithTime();
        if (odom.HasData() && odom.data->stamp_ns != last_stamp) {
            last_stamp = odom.data->stamp_ns;
            ++frames;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window_start < std::chrono::seconds(1))
            continue;
        window_start = now;

        if (!odom.HasData()) {
            std::printf("no odometry (buffer empty)\n");
            frames = 0;
            continue;
        }

        const auto& o = *odom.data;
        const float yaw = std::atan2(2.0f * (o.qw * o.qz + o.qx * o.qy),
                                     1.0f - 2.0f * (o.qy * o.qy + o.qz * o.qz));
        std::printf("rate=%3dHz  age=%6.1fms  frame_id=%s  pos=(%.2f, %.2f, %.2f)  yaw=%.1fdeg  vx=%.2f\n",
                    frames, odom.GetAgeMs(), o.frame_id.c_str(),
                    o.px, o.py, o.pz, yaw * 57.29578f, o.vx);
        frames = 0;
    }

    reader.stop();
    return 0;
}
