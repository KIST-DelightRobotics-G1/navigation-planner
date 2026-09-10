// Live check of the LioReceiver: with the LIO engine up (docs/LIO_ENGINE.md), it
// subscribes to both engine outputs and prints, at ~5 Hz, the pose (T_odom_lidar:
// pos + yaw + velocity) and the registered cloud (point count + odom bounding box).
// Proves the planner consumes the engine ROS-free through one receiver block.
//   ./test_lio_receiver [config.yaml]

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "lio/lio_receiver.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string cfg = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(cfg);
    const auto& root = kist::Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;

    std::signal(SIGINT, [](int) { g_stop = true; });

    kist::LioReceiver rx;
    if (!rx.start(domain)) return 1;
    std::printf("waiting for the LIO engine (/Odometry_loc + /cloud_registered_1) — Ctrl+C\n");

    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        auto od = rx.odom_buf.GetData();
        auto cl = rx.cloud_buf.GetData();

        if (od) {
            const auto& q = od->pose.T_parent_child.rotation;
            const auto& t = od->pose.T_parent_child.translation;
            const double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())) * 57.2958;
            std::printf("pose pos=(% .3f % .3f % .3f) yaw=% 6.1f  vel=(% .2f % .2f % .2f)",
                        t.x(), t.y(), t.z(), yaw,
                        od->linear_velocity.x(), od->linear_velocity.y(), od->linear_velocity.z());
        } else {
            std::printf("pose --");
        }

        if (cl && cl->point_count() > 0) {
            double lo[3] = { 1e30, 1e30, 1e30 }, hi[3] = { -1e30, -1e30, -1e30 };
            const std::size_t n = cl->point_count();
            for (std::size_t i = 0; i < n; ++i)
                for (int k = 0; k < 3; ++k) {
                    const double v = cl->xyz[3 * i + k];
                    if (v < lo[k]) lo[k] = v;
                    if (v > hi[k]) hi[k] = v;
                }
            std::printf(" | cloud[%s] n=%5zu x[% .1f,% .1f] y[% .1f,% .1f] z[% .1f,% .1f]",
                        cl->frame_id.c_str(), n, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]);
        } else {
            std::printf(" | cloud --");
        }
        std::printf("\n");
    }

    rx.stop();
    return 0;
}
