// Live UWB + odometry fusion check (robot LAN + UWB transmitter running):
//   ./test_pose_filter [config_path]
// Starts the odometry reader and the (embedded) UwbReceiver, feeds them into
// PoseFilter exactly as the future orchestrator will, and prints the fused
// global pose once per second. Ctrl-C to stop.
//
// The EKF needs motion to calibrate its yaw bias: until b_theta converges
// (<5deg std) calibrated_pose_buf stays empty and this prints "calibrating" — drive the
// robot around and the pose starts publishing.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "kalman_filter/pose_filter.hpp"
#include "unitree/unitree_odometry_reader.hpp"
#include "system/uwb_receiver.hpp"   // embedded from kist-ext-sensor-io
#include "uwb/uwb_position.hpp"

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
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);

    const auto unitree_cfg = Config::instance().root()["unitree"];
    const int         domain_id = unitree_cfg["domain_id"].as<int>();
    if (!apply_dds_config(Config::instance().root())) return 1;  // NIC + tuning from config/cyclonedds.xml

    const auto pf_cfg = Config::instance().root()["pose_filter"];
    const auto options = pose_filter_options_from_yaml(pf_cfg);
    const std::string frame_id = options.frame_id;

    PoseFilter filter(options);

    // UWB fixes are pushed into the filter's input buffer by the callback.
    UwbReceiver uwb;
    uwb.set_on_position([&](const UwbPosition& p) {
        filter.uwb_buf.SetData(UwbFix{p.stamp_ns, p.x, p.y});
    });

    auto& odom = UnitreeOdometryReader::instance();
    if (!uwb.start(domain_id, "")) return 1;
    if (!odom.start(domain_id, "")) return 1;

    // The filter pulls odom straight from the reader's buffer (no hook).
    filter.start(odom.odom_buf);

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_pose_filter] domain=%d frame=%s - move the robot to calibrate\n",
                domain_id, frame_id.c_str());

    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const bool have_odom = odom.odom_buf.GetData() != nullptr;
        const bool have_uwb  = filter.uwb_buf.GetData() != nullptr;
        auto pose = filter.calibrated_pose_buf.GetDataWithTime();

        if (pose.HasData() && pose.GetAgeMs() < 1000.0) {
            const auto& p = *pose.data;
            std::printf("pose  (%.2f, %.2f)  yaw=%.1fdeg  frame=%s\n",
                        p.x, p.y, p.yaw * 180.0f / float(M_PI), p.frame_id.c_str());
        } else {
            std::printf("calibrating  odom=%s uwb=%s  (no trusted pose yet - move the robot)\n",
                        have_odom ? "ok" : "--", have_uwb ? "ok" : "--");
        }
    }

    filter.stop();
    odom.stop();
    uwb.stop();
    return 0;
}
