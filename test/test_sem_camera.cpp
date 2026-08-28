// Live camera semantic segmentation (robot LAN + RealSense transmitter running):
//   ./test_sem_camera [config_path]
// Wires the embedded RealsenseReceiver (H.264 color over DDS) into a
// YoloPipeline<YoloSemSegEngine>, which runs YOLO26-sem on its own worker thread
// (latest-wins) producing a dense per-pixel class map, and overlays a colorized
// class map on the live video. Prints the fps + top class ids (by pixel share)
// once per second (names TBD until we confirm the model's class set). With a
// DISPLAY it opens a window (ESC); headless it writes /tmp/sem_camera_out.png.

#include "cv/yolo/yolo_pipeline.hpp"
#include "cv/yolo/semantic_segmentation/yolo_sem_seg_engine.hpp"
#include "system/realsense_receiver.hpp"   // embedded from kist-ext-sensor-io
#include "common/config.hpp"
#include "common/dds_config.hpp"            // apply_dds_config (kist-ext-sensor-io)

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <array>
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

// A distinct color per class id (deterministic HSV hue), as a 256-entry LUT.
// Class ids span only a small range, so a continuous colormap (JET) would put
// them all at one end — a per-id palette keeps classes visually separable.
const cv::Mat& class_palette() {
    static const cv::Mat lut = [] {
        cv::Mat p(256, 1, CV_8UC3);
        for (int i = 0; i < 256; ++i) {
            cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar((i * 47) % 180, 200, 255)), bgr;
            cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
            p.at<cv::Vec3b>(i, 0) = bgr.at<cv::Vec3b>(0, 0);
        }
        p.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 0);   // id 0 -> black (often bg)
        return p;
    }();
    return lut;
}

// Colorized class map cropped to the letterbox content region and resized to the
// original frame, then blended in.
void draw(cv::Mat& vis, const cv::Mat& base, const SemSegResult& r) {
    if (r.class_map.empty()) { base.copyTo(vis); return; }

    const int nw = int(std::round(r.width * r.scale));
    const int nh = int(std::round(r.height * r.scale));
    cv::Rect content(r.pad_x, r.pad_y, nw, nh);
    content &= cv::Rect(0, 0, r.class_map.cols, r.class_map.rows);
    if (content.width <= 0 || content.height <= 0) { base.copyTo(vis); return; }

    cv::Mat cls_orig;
    cv::resize(r.class_map(content), cls_orig, base.size(), 0, 0, cv::INTER_NEAREST);
    cv::Mat cls3, color;
    cv::cvtColor(cls_orig, cls3, cv::COLOR_GRAY2BGR);   // replicate id to 3 channels
    cv::LUT(cls3, class_palette(), color);              // per-id distinct color
    cv::addWeighted(color, 0.5, base, 0.5, 0, vis);
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);
    const auto& root = Config::instance().root();

    const int domain_id = root["unitree"]["domain_id"].as<int>(0);
    // Route config/cyclonedds.xml (NIC + rx buffer/defrag tuning) into CycloneDDS.
    // Must precede any receiver start(), which is then passed an EMPTY interface.
    if (!apply_dds_config(root)) return 1;

    YoloSemSegConfig cfg;
    double      target_fps = 30.0;
    std::string cam_name   = "head";
    if (const auto cv = Config::instance().root()["cv_inference"]) {
        cfg.onnx_path = cv["semantic_onnx"].as<std::string>(cfg.onnx_path);
        target_fps    = cv["target_fps"].as<double>(target_fps);
        cam_name      = cv["camera"].as<std::string>(cam_name);
    }

    RealsenseReceiver rx;
    if (!rx.start(domain_id, "", cam_name)) return 1;   // empty iface — NIC from the DDS xml

    YoloPipeline<YoloSemSegEngine> pipe;
    if (!pipe.start(cfg, target_fps, [&](cv::Mat& bgr, int64_t& stamp) -> bool {
        auto cf = rx.color().GetData();
        if (!cf || cf->empty()) return false;
        stamp = cf->stamp_ns;
        cv::Mat(cf->height, cf->width, CV_8UC3,
                const_cast<uint8_t*>(cf->data.data()), cf->stride_bytes).copyTo(bgr);
        return true;
    })) return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    const bool has_disp = [] { const char* e = std::getenv("DISPLAY"); return e && e[0]; }();
    std::printf("[test_sem_camera] domain=%d cap=%.0ffps - %s\n",
                domain_id, target_fps,
                has_disp ? "window (ESC to quit)" : "headless -> /tmp/sem_camera_out.png");

    int64_t  last_result_stamp = -1;
    uint64_t last_processed    = 0;
    auto     log_window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto cf  = rx.color().GetData();
        auto res = pipe.result().GetData();

        if (cf && !cf->empty()) {
            cv::Mat bgr(cf->height, cf->width, CV_8UC3,
                        const_cast<uint8_t*>(cf->data.data()), cf->stride_bytes);
            cv::Mat vis = bgr.clone();
            if (res) draw(vis, bgr, *res);
            if (has_disp) {
                cv::imshow("YOLO26-sem (live)", vis);
                if (cv::waitKey(30) == 27) break;
            } else {
                cv::imwrite("/tmp/sem_camera_out.png", vis);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } else if (has_disp) {
            if (cv::waitKey(30) == 27) break;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - log_window >= std::chrono::seconds(1)) {
            log_window = now;
            const uint64_t processed = pipe.frames_processed();
            const auto t = pipe.timings();
            std::printf("sem %2llu fps  [pre %.1f  infer %.1f  post %.1f ms]  %s\n",
                        (unsigned long long)(processed - last_processed),
                        t.preprocess_ms, t.infer_ms, t.postprocess_ms,
                        cf ? "" : "(no camera)");
            last_processed = processed;

            // Top class ids by pixel share (names TBD until the class set is known).
            if (res && res->stamp_ns != last_result_stamp && !res->class_map.empty()) {
                last_result_stamp = res->stamp_ns;
                std::array<uint64_t, 256> hist{};
                const cv::Mat& m = res->class_map;
                for (int y = 0; y < m.rows; ++y) {
                    const uint8_t* row = m.ptr<uint8_t>(y);
                    for (int x = 0; x < m.cols; ++x) ++hist[row[x]];
                }
                const uint64_t total = uint64_t(m.rows) * m.cols;
                std::array<int, 256> ids{};
                for (int i = 0; i < 256; ++i) ids[i] = i;
                std::sort(ids.begin(), ids.end(),
                          [&](int a, int b) { return hist[a] > hist[b]; });
                std::printf("    top classes:");
                for (int k = 0; k < 6 && hist[ids[k]] > 0; ++k)
                    std::printf("  id%d=%.0f%%", ids[k], 100.0 * hist[ids[k]] / total);
                std::printf("\n");
            }
        }
    }

    pipe.stop();
    rx.stop();
    return 0;
}
