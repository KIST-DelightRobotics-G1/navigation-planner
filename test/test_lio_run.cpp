// Live LIO runner — the first real-data run. Starts the Mid-360 cloud + IMU readers,
// runs the ported FAST-LIO worker, and prints T_odom_lidar at ~5 Hz. This is the
// Stage 6 verification tool: stand still -> pose steady; move the sensor -> pose
// tracks; walk -> trajectory grows. Ctrl+C to quit.
//   ./test_lio_run [config.yaml]

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "lio/lio_runtime.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
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

    kist::LioConfig lcfg;   // defaults; extrinsic identity (Mid-360 IMU ~ lidar,
                            // FAST-LIO refines it online). Tune from config later.
    kist::LioRuntime rt;
    if (!rt.start(domain, lcfg)) return 1;

    std::printf("running LIO on live Mid-360 (Ctrl+C to quit)...\n"
                "stand still -> pose steady; move -> pose tracks.\n");

    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto p = rt.pose_buf.GetData();
        if (!p) {
            std::printf("no valid pose yet (processed=%llu)\n",
                        (unsigned long long)rt.frames_processed.load());
            continue;
        }
        const auto& q = p->orientation;
        const double yaw = std::atan2(2.0 * (q.w()*q.z() + q.x()*q.y()),
                                      1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z())) * 57.2958;
        std::printf("T_odom_lidar: pos=(% .3f % .3f % .3f) yaw=% 6.1f deg  "
                    "vel=(% .2f % .2f % .2f) | processed=%llu dropped=%llu\n",
                    p->position.x(), p->position.y(), p->position.z(), yaw,
                    p->linear_velocity.x(), p->linear_velocity.y(), p->linear_velocity.z(),
                    (unsigned long long)rt.frames_processed.load(),
                    (unsigned long long)rt.frames_dropped.load());
    }

    rt.stop();
    return 0;
}
