// Live LiDAR point-cloud viewer — top-down (bird's-eye) projection.
//   ./test_pointcloud_viewer [config_path]        (default config/config.yaml)
// Subscribes to the robot's Livox cloud via UnitreePointCloudReader (with the
// config's pointcloud_processor transform+filter applied), and renders a top-
// down X-Y image colored by height Z. Robot at center: +X (forward) is up,
// +Y (left) is left; grid rings every 2 m. With a DISPLAY it opens a window
// (ESC to quit); headless it writes /tmp/pointcloud_view.png once per second.
// Prints point count + received fps once per second.

#include "unitree/unitree_pointcloud_reader.hpp"
#include "pointcloud/pointcloud_processor.hpp"
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

namespace {
constexpr double kRangeM = 10.0;              // ± view range (m) in X and Y
constexpr int    kImg    = 800;               // square image, px
constexpr double kZMin   = -0.5, kZMax = 2.0; // height color range (m)
const cv::Scalar kBg(30, 30, 30), kGrid(60, 60, 60), kAxis(95, 95, 95), kText(225, 225, 225);

// 256-entry JET LUT for height coloring.
const cv::Mat& jet_lut() {
    static const cv::Mat lut = [] {
        cv::Mat g(256, 1, CV_8U);
        for (int i = 0; i < 256; ++i) g.at<uchar>(i, 0) = uchar(i);
        cv::Mat c; cv::applyColorMap(g, c, cv::COLORMAP_JET);
        return c;
    }();
    return lut;
}

cv::Mat render(const UnitreePointCloud& c, int fps) {
    cv::Mat img(kImg, kImg, CV_8UC3, kBg);
    const double ppm = kImg / (2.0 * kRangeM);   // pixels per meter
    const int cx = kImg / 2, cy = kImg / 2;

    // range rings (every 2 m) + axes
    for (int r = 2; r <= int(kRangeM); r += 2)
        cv::circle(img, {cx, cy}, int(r * ppm), kGrid, 1, cv::LINE_AA);
    cv::line(img, {cx, 0}, {cx, kImg}, kAxis, 1);
    cv::line(img, {0, cy}, {kImg, cy}, kAxis, 1);

    // points, colored by height Z
    const float* p = c.xyz.data();
    const size_t n = c.point_count();
    for (size_t i = 0; i < n; ++i) {
        const float x = p[3 * i], y = p[3 * i + 1], z = p[3 * i + 2];
        const int col = cx - int(std::lround(y * ppm));   // +Y (left) -> left
        const int row = cy - int(std::lround(x * ppm));   // +X (fwd)  -> up
        if (col < 0 || col >= kImg || row < 0 || row >= kImg) continue;
        const double t = std::clamp((z - kZMin) / (kZMax - kZMin), 0.0, 1.0);
        img.at<cv::Vec3b>(row, col) = jet_lut().at<cv::Vec3b>(int(t * 255), 0);
    }

    cv::circle(img, {cx, cy}, 4, cv::Scalar(0, 0, 255), -1);   // robot at origin
    char label[96];
    std::snprintf(label, sizeof label, "%zu pts  %d fps  (+-%.0fm, height %.1f..%.1fm)",
                  n, fps, kRangeM, kZMin, kZMax);
    cv::putText(img, label, {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.45, kText, 1, cv::LINE_AA);
    return img;
}
}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);
    const auto& root = Config::instance().root();

    const int domain_id = root["unitree"]["domain_id"].as<int>(0);
    if (!apply_dds_config(root)) return 1;   // NIC + rx tuning from config/cyclonedds.xml

    auto& reader = UnitreePointCloudReader::instance();
    reader.set_processor(
        [proc = PointCloudProcessor{
             pointcloud_processor_options_from_yaml(root["pointcloud_processor"])}](
            UnitreePointCloud& cloud) mutable { proc.process(cloud); });
    if (!reader.start(domain_id, "")) {   // empty iface — NIC from the DDS xml
        std::fprintf(stderr, "[pointcloud_viewer] reader start failed\n");
        return 1;
    }

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    const bool has_disp = [] { const char* e = std::getenv("DISPLAY"); return e && e[0]; }();
    std::printf("[pointcloud_viewer] domain=%d - %s\n", domain_id,
                has_disp ? "window (ESC to quit)" : "headless -> /tmp/pointcloud_view.png");

    int64_t last_stamp = -1;
    int     recv_n = 0, fps = 0;
    auto    window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto cloud = reader.cloud_buf.GetData();
        if (cloud && cloud->stamp_ns != last_stamp) { last_stamp = cloud->stamp_ns; ++recv_n; }

        if (has_disp) {
            cv::imshow("LiDAR (top-down)", render(cloud ? *cloud : UnitreePointCloud{}, fps));
            if (cv::waitKey(30) == 27) break;   // ESC
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            fps = recv_n; recv_n = 0;
            std::printf("  %zu pts  %d fps\n", cloud ? cloud->point_count() : 0, fps);
            if (!has_disp)
                cv::imwrite("/tmp/pointcloud_view.png",
                            render(cloud ? *cloud : UnitreePointCloud{}, fps));
        }
    }

    reader.stop();
    return 0;
}
