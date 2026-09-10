// Live check of the LIO -> transform tree wiring. With the engine up (docs/LIO_ENGINE.md):
// the LioReceiver gets T_odom_lidar_imu, LioTreeProducer feeds it as the odom->lidar_imu
// dynamic edge (plus the static lidar_imu->lidar), and we lookup(Odom, Lidar) to get the
// composed T_odom_lidar. Prints the imu-body pos and the composed lidar pos side by side —
// they differ by the fixed ~4 cm imu->lidar offset (rotated by the current orientation).
//   ./test_lio_tree [config.yaml]

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "lio/lio_receiver.hpp"
#include "lio/lio_tree_producer.hpp"
#include "transforms/transform_tree.hpp"

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

    kist::TransformTree tree;
    kist::LioTreeProducer producer(tree);   // sets the static lidar_imu->lidar edge

    std::printf("waiting for the LIO engine — Ctrl+C\n");

    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto od = rx.odom_buf.GetData();
        if (!od) { std::printf("no odom yet...\n"); continue; }

        producer.update(*od);   // odom->lidar_imu dynamic edge

        auto T = tree.lookupTransform(kist::FrameId::Odom, kist::FrameId::Lidar, od->pose.stamp_ns);
        if (!T) { std::printf("lookup gap (edge unset / stamp out of history)\n"); continue; }

        const auto& imu = od->pose.T_parent_child.translation;   // T_odom_lidar_imu
        const auto& lid = T->translation;                        // T_odom_lidar (composed)
        const auto& q = T->rotation;
        const double yaw = std::atan2(2.0 * (q.w()*q.z() + q.x()*q.y()),
                                      1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z())) * 57.2958;
        std::printf("imu_body=(% .3f % .3f % .3f)  ->  lidar=(% .3f % .3f % .3f)  yaw=% 6.1f  d=%.3fm\n",
                    imu.x(), imu.y(), imu.z(), lid.x(), lid.y(), lid.z(), yaw,
                    (lid - imu).norm());
    }

    rx.stop();
    return 0;
}
