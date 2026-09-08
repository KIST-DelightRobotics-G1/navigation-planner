// Visual live check for PoseFilter (robot LAN + UWB transmitter running):
//   ./test_pose_filter_viewer [config_path]
// Same wiring as test_pose_filter, plus a top-down OpenCV plot of the UWB frame:
//   * blue dot   — fused pose (PoseFilter output, calibrated_pose_buf)
//   * blue arrow — fused yaw (heading)
//   * red X      — raw UWB fix (uwb_buf, pre-fusion)
// Numeric values + calibration/gating state go to the log, once per second.
// With a DISPLAY it opens a window (ESC to quit); headless it writes
// /tmp/pose_filter_view.png periodically.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "kalman_filter/pose_filter.hpp"
#include "unitree/unitree_odometry_reader.hpp"
#include "system/uwb_receiver.hpp"   // embedded from kist-ext-sensor-io
#include "uwb/uwb_position.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <deque>
#include <string>
#include <thread>
#include <utility>

using namespace kist;

static std::atomic<bool> g_stop{false};

namespace {
constexpr int    kCanvas = 900;      // px, square
constexpr double kScale  = 60.0;     // px per metre
constexpr int    kPoseTrail   = 400; // fused-pose path length (points)
constexpr double kUwbTrailSec = 1.5; // raw-UWB X marks fade out over this window (s)
const cv::Scalar kBlue(255, 0, 0), kRed(0, 0, 255), kGrid(60, 60, 60),
                 kBg(30, 30, 30), kText(220, 220, 220);
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);

    const auto unitree_cfg = Config::instance().root()["unitree"];
    const int         domain_id = unitree_cfg["domain_id"].as<int>();
    if (!apply_dds_config(Config::instance().root())) return 1;  // NIC + tuning from config/cyclonedds.xml

    const auto options = pose_filter_options_from_yaml(Config::instance().root()["pose_filter"]);

    PoseFilter filter(options);

    UwbReceiver uwb;
    uwb.set_on_position([&](const UwbPosition& p) {
        filter.uwb_buf.SetData(UwbFix{p.stamp_ns, p.x, p.y});
    });
    auto& odom = UnitreeOdometryReader::instance();
    if (!uwb.start(domain_id, "")) return 1;
    if (!odom.start(domain_id, "")) return 1;
    filter.start(odom.odom_buf);

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    const bool has_disp = [] { const char* d = std::getenv("DISPLAY"); return d && d[0]; }();
    std::printf("[test_pose_filter_viewer] domain=%d frame=%s - %s (move the robot to calibrate)\n",
                domain_id, options.frame_id.c_str(),
                has_disp ? "window" : "headless -> /tmp/pose_filter_view.png");

    // Fix the view centre on the first datum so the trails stay put in world space.
    bool   have_centre = false;
    double cx = 0.0, cy = 0.0;
    auto to_px = [&](float wx, float wy) {
        return cv::Point(int(kCanvas / 2 + (wx - cx) * kScale),
                         int(kCanvas / 2 - (wy - cy) * kScale));  // y up
    };

    std::deque<cv::Point2f> pose_trail;
    std::deque<std::pair<cv::Point2f, std::chrono::steady_clock::time_point>> uwb_trail;
    int64_t last_pose_stamp = -1, last_uwb_stamp = -1;
    auto log_window = std::chrono::steady_clock::now();

    while (!g_stop) {
        const auto frame_now = std::chrono::steady_clock::now();
        auto pose = filter.calibrated_pose_buf.GetDataWithTime();
        auto fix  = filter.uwb_buf.GetData();

        // Seed the view centre.
        if (!have_centre) {
            if (pose.HasData())      { cx = pose.data->x; cy = pose.data->y; have_centre = true; }
            else if (fix)            { cx = fix->x;       cy = fix->y;       have_centre = true; }
        }

        // Grow trails on new samples. The raw-UWB X marks are time-bounded so
        // stale fixes fade out fast (they otherwise pile up and mislead); the
        // fused-pose path is a plain length-bounded trail.
        if (fix && fix->stamp_ns != last_uwb_stamp) {
            last_uwb_stamp = fix->stamp_ns;
            uwb_trail.emplace_back(cv::Point2f(fix->x, fix->y), frame_now);
        }
        while (!uwb_trail.empty() &&
               std::chrono::duration<double>(frame_now - uwb_trail.front().second).count()
                   > kUwbTrailSec)
            uwb_trail.pop_front();

        const bool pose_live = pose.HasData() && pose.GetAgeMs() < 1000.0;
        if (pose_live && pose.data->stamp_ns != last_pose_stamp) {
            last_pose_stamp = pose.data->stamp_ns;
            pose_trail.emplace_back(pose.data->x, pose.data->y);
            if (pose_trail.size() > kPoseTrail) pose_trail.pop_front();
        }

        // ── draw ──
        cv::Mat canvas(kCanvas, kCanvas, CV_8UC3, kBg);
        for (int m = -7; m <= 7; ++m) {          // 1m grid
            cv::line(canvas, to_px(float(cx + m), float(cy - 8)),
                     to_px(float(cx + m), float(cy + 8)), kGrid, 1);
            cv::line(canvas, to_px(float(cx - 8), float(cy + m)),
                     to_px(float(cx + 8), float(cy + m)), kGrid, 1);
        }
        for (auto& [wp, t] : uwb_trail) {        // red X marks (raw UWB), fading by age
            const double age = std::chrono::duration<double>(frame_now - t).count();
            const double a = 1.0 - age / kUwbTrailSec;          // 1=new .. 0=old
            const cv::Scalar col = kBg + (kRed - kBg) * (a < 0.0 ? 0.0 : a);
            cv::Point c = to_px(wp.x, wp.y);
            cv::line(canvas, c + cv::Point(-4, -4), c + cv::Point(4, 4), col, 1);
            cv::line(canvas, c + cv::Point(-4, 4), c + cv::Point(4, -4), col, 1);
        }
        for (size_t i = 1; i < pose_trail.size(); ++i)   // blue path
            cv::line(canvas, to_px(pose_trail[i - 1].x, pose_trail[i - 1].y),
                     to_px(pose_trail[i].x, pose_trail[i].y), cv::Scalar(150, 60, 60), 1);

        std::string status;
        if (pose_live) {
            const auto& p = *pose.data;
            cv::Point c = to_px(p.x, p.y);
            cv::circle(canvas, c, 6, kBlue, cv::FILLED);            // fused pose
            cv::Point tip(c.x + int(std::cos(p.yaw) * 0.6 * kScale),
                          c.y - int(std::sin(p.yaw) * 0.6 * kScale));
            cv::arrowedLine(canvas, c, tip, kBlue, 2, cv::LINE_AA, 0, 0.3);  // yaw
            char buf[128];
            std::snprintf(buf, sizeof buf, "pose (%.2f, %.2f)  yaw=%.1f deg",
                          p.x, p.y, p.yaw * 180.0 / M_PI);
            status = buf;
        } else {
            status = "no trusted pose (calibrating / gated - see log)";
        }
        cv::putText(canvas, status, {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.6, kText, 1, cv::LINE_AA);
        cv::putText(canvas, "blue=fused pose/yaw   red X=raw UWB", {12, kCanvas - 16},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, kText, 1, cv::LINE_AA);

        if (has_disp) {
            cv::imshow("pose filter: fused pose (blue) vs raw UWB (red)", canvas);
            if (cv::waitKey(30) == 27) break;    // ESC
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Log values + state once per second (this is where numbers live).
        const auto now = std::chrono::steady_clock::now();
        if (now - log_window >= std::chrono::seconds(1)) {
            log_window = now;
            if (!has_disp) cv::imwrite("/tmp/pose_filter_view.png", canvas);
            if (pose_live)
                std::printf("pose=(%.2f, %.2f) yaw=%.1fdeg   uwb=(%.2f, %.2f)\n",
                            pose.data->x, pose.data->y, pose.data->yaw * 180.0 / M_PI,
                            fix ? fix->x : 0.f, fix ? fix->y : 0.f);
            else
                std::printf("no trusted pose   uwb=%s\n", fix ? "ok" : "--");
        }
    }

    filter.stop();
    odom.stop();
    uwb.stop();
    return 0;
}
