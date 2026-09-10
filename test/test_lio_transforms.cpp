// Live check of LioTransformProducer, formally wired: the producer runs on the LIO
// receive thread (via LioReceiver's per-frame hook), reading the latest lowstate and
// composing T_odom_lidar (sensor/head) + T_odom_pelvis (robot base via waist FK). The
// main loop only READS the output buffer (like a real consumer) and prints both + waist.
//
// WALK the robot: T_odom_lidar wobbles with the head sway; T_odom_pelvis follows the true
// base motion (real stagger, minus the head-sway artifact). See docs / lio_transform_producer.
//   ./test_lio_transforms [config.yaml]

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "lio/lio_receiver.hpp"
#include "lio/lio_transform_producer.hpp"
#include "unitree/unitree_state_reader.hpp"

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

    kist::LioTransformProducer prod;
    auto& sr = kist::UnitreeStateReader::instance();
    if (!sr.start(domain, "")) return 1;

    // Formal wiring: each LIO frame (on the LIO Rx thread) drives the producer, which
    // reads the latest lowstate and publishes RobotTransforms to prod.out_buf.
    kist::LioReceiver rx;
    rx.set_odom_hook([&](const kist::LioOdometry& od) {
        if (auto st = sr.state_buf.GetData()) prod.step(od, *st);
    });
    if (!rx.start(domain)) return 1;

    std::printf("WALK the robot: lidar wobbles, pelvis = true base motion (Ctrl+C)\n");

    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto rt = prod.out_buf.GetData();
        if (!rt) { std::printf("no transforms yet (LIO engine + lowstate up?)...\n"); continue; }

        const auto& L = rt->T_odom_lidar.translation;
        const auto& P = rt->T_odom_pelvis.translation;
        const auto& q = rt->T_odom_lidar.rotation;
        const double C = 57.2958;
        const double lroll  = std::atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(q.x()*q.x()+q.y()*q.y())) * C;
        const double lpitch = std::asin(std::max(-1.0, std::min(1.0, 2*(q.w()*q.y()-q.z()*q.x())))) * C;
        const double lyaw   = std::atan2(2*(q.w()*q.z()+q.x()*q.y()), 1-2*(q.y()*q.y()+q.z()*q.z())) * C;

        auto st = sr.state_buf.GetData();
        const double wy = st ? st->motors[12].q * C : 0.0;
        const double wr = st ? st->motors[13].q * C : 0.0;
        const double wp = st ? st->motors[14].q * C : 0.0;
        std::printf("lidar_pos=(% .3f % .3f % .3f) rpy=(% .0f % .0f % .0f) | pelvis=(% .3f % .3f % .3f) | waist=% .0f/% .0f/% .0f\n",
                    L.x(), L.y(), L.z(), lroll, lpitch, lyaw, P.x(), P.y(), P.z(), wy, wr, wp);
    }

    rx.stop();
    sr.stop();
    return 0;
}
