// Grab one color frame from the embedded RealsenseReceiver (H.264 over DDS) and
// save it to disk — a clean, un-annotated frame for feeding to model spikes
// (e.g. the OneFormer indoor-quality check) or for offline debugging.
//   ./test_camera_snapshot [config_path] [out.png] [camera_name]
// Defaults: config/config.yaml, capture.png (repo root, so it lands on the host
// via the bind mount), and segmentation.camera (or "head"). Needs the camera
// transmitter to be publishing.

#include "system/realsense_receiver.hpp"   // embedded from kist-ext-sensor-io
#include "common/config.hpp"

#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

using namespace kist;

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    const std::string out_path    = (argc >= 3) ? argv[2] : "capture.png";

    Config::instance().load(config_path);
    const auto& root = Config::instance().root();
    const auto unitree      = root["unitree"];
    const int domain_id     = unitree["domain_id"].as<int>(0);
    const std::string iface = unitree["network_interface"].as<std::string>("lo");

    std::string cam_name = "head";
    if (const auto cv = root["cv_inference"]) cam_name = cv["camera"].as<std::string>(cam_name);
    if (argc >= 4) cam_name = argv[3];

    RealsenseReceiver rx;
    if (!rx.start(domain_id, iface, cam_name)) {
        std::fprintf(stderr, "[snapshot] receiver start failed\n");
        return 1;
    }
    std::printf("[snapshot] waiting for a color frame on '%s' (domain=%d iface=%s) ...\n",
                cam_name.c_str(), domain_id, iface.c_str());

    // Wait up to ~5s for the first decoded color frame.
    for (int i = 0; i < 500; ++i) {
        if (auto c = rx.color().GetData(); c && !c->empty()) {
            cv::Mat bgr(c->height, c->width, CV_8UC3,
                        const_cast<uint8_t*>(c->data.data()), c->stride_bytes);
            if (cv::imwrite(out_path, bgr)) {
                std::printf("[snapshot] saved %dx%d -> %s\n", c->width, c->height, out_path.c_str());
                rx.stop();
                return 0;
            }
            std::fprintf(stderr, "[snapshot] imwrite failed: %s\n", out_path.c_str());
            rx.stop();
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::fprintf(stderr, "[snapshot] no frame within 5s — is the transmitter publishing?\n");
    rx.stop();
    return 1;
}
