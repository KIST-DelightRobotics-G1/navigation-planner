// Live camera segmentation (robot LAN + RealSense transmitter running):
//   ./test_seg_camera [config_path]        (default config/config.yaml)
// Wires the embedded RealsenseReceiver (H.264 color over DDS) into SegInference,
// which runs YOLO26-seg on its own worker thread (~10 fps cap, latest-wins), and
// overlays the latest masks on the live video. With a DISPLAY it opens a window
// (ESC to quit); headless it writes /tmp/seg_camera_out.png. Detection rate is
// logged once per second.
//
// The camera transmitter must be publishing (ext-sensor-io test_realsense_transmitter
// on the robot; free the device first with scripts/free_camera.sh). First run
// builds+caches the .trt engine.

#include "segmentation/segment_inference.hpp"
#include "system/realsense_receiver.hpp"   // embedded from kist-ext-sensor-io
#include "common/config.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

namespace {
const char* kCoco[80] = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush"};
const char* name_of(int id) { return (id >= 0 && id < 80) ? kCoco[id] : "?"; }

cv::Scalar color_for(int id) {
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar((id * 47) % 180, 200, 255)), bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    cv::Vec3b c = bgr.at<cv::Vec3b>(0, 0);
    return cv::Scalar(c[0], c[1], c[2]);
}

void draw(cv::Mat& vis, const cv::Mat& base, const SegResult& r) {
    for (const auto& d : r.detections) {
        const cv::Scalar col = color_for(d.class_id);
        cv::Mat colored(base.size(), CV_8UC3, col);
        colored.copyTo(vis, d.mask);
        cv::rectangle(vis, d.box, col, 2);
        char label[64];
        std::snprintf(label, sizeof label, "%s %.2f", name_of(d.class_id), d.score);
        cv::putText(vis, label, {d.box.x, std::max(0, d.box.y - 5)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv::LINE_AA);
    }
    cv::addWeighted(vis, 0.6, base, 0.4, 0, vis);
}
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);

    const auto unitree_cfg = Config::instance().root()["unitree"];
    const int         domain_id = unitree_cfg["domain_id"].as<int>();
    const std::string interface = unitree_cfg["network_interface"].as<std::string>();

    YoloSegConfig scfg;
    double      target_fps = 10.0;
    std::string cam_name   = "head";   // which camera's topics to segment
    if (const auto sc = Config::instance().root()["segmentation"]) {
        scfg.onnx_path = sc["onnx_path"].as<std::string>(scfg.onnx_path);
        target_fps     = sc["target_fps"].as<double>(target_fps);
        cam_name       = sc["camera"].as<std::string>(cam_name);
    }

    RealsenseReceiver rx;
    if (!rx.start(domain_id, interface, cam_name)) return 1;

    SegInference seg;
    if (!seg.init(scfg, target_fps)) return 1;
    seg.start([&](cv::Mat& bgr, int64_t& stamp) -> bool {
        auto cf = rx.color().GetData();
        if (!cf || cf->empty()) return false;
        stamp = cf->stamp_ns;
        cv::Mat(cf->height, cf->width, CV_8UC3,
                const_cast<uint8_t*>(cf->data.data()), cf->stride_bytes).copyTo(bgr);
        return true;
    });

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    const bool has_disp = [] { const char* e = std::getenv("DISPLAY"); return e && e[0]; }();
    std::printf("[test_seg_camera] domain=%d iface=%s cap=%.0ffps - %s\n",
                domain_id, interface.c_str(), target_fps,
                has_disp ? "window (ESC to quit)" : "headless -> /tmp/seg_camera_out.png");

    int64_t last_result_stamp = -1;
    int     seg_frames = 0, dets = 0;
    auto    log_window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto cf  = rx.color().GetData();
        auto res = seg.result_buf.GetData();

        if (cf && !cf->empty()) {
            cv::Mat bgr(cf->height, cf->width, CV_8UC3,
                        const_cast<uint8_t*>(cf->data.data()), cf->stride_bytes);
            cv::Mat vis = bgr.clone();
            if (res) draw(vis, bgr, *res);
            if (has_disp) {
                cv::imshow("YOLO26-seg (live)", vis);
                if (cv::waitKey(30) == 27) break;          // ~30fps display, ESC to quit
            } else {
                cv::imwrite("/tmp/seg_camera_out.png", vis);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } else if (has_disp) {
            if (cv::waitKey(30) == 27) break;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Count segmentation throughput (distinct result stamps).
        if (res && res->stamp_ns != last_result_stamp) {
            last_result_stamp = res->stamp_ns;
            ++seg_frames;
            dets += int(res->detections.size());
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - log_window >= std::chrono::seconds(1)) {
            log_window = now;
            std::printf("seg %2d fps  %d det/frame avg  %s\n", seg_frames,
                        seg_frames ? dets / seg_frames : 0, cf ? "" : "(no camera)");
            seg_frames = 0; dets = 0;
        }
    }

    seg.stop();
    rx.stop();
    return 0;
}
