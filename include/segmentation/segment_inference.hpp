#pragma once

#include "common/data_buffer.hpp"
#include "segmentation/seg_result.hpp"
#include "segmentation/yolo/yolo_seg_engine.hpp"

#include <opencv2/core.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>

namespace kist {

// Segmentation inference stage — owns a YoloSegEngine and runs it on its own
// worker thread, decoupled from the camera source. The engine (a Module) does
// the compute; this Thread pulls the latest frame at a capped rate (latest-
// wins, so it never backs up behind a faster camera) and publishes results.
//
// Frames come via an injected FrameSource callback rather than a concrete
// camera type, so this stage builds without ext-sensor-io: the wiring layer
// (a runner, or the future orchestrator) supplies a lambda that reads
// RealsenseReceiver.color() and hands over an owning BGR copy + stamp.
//
// Wiring:
//   SegInference seg;
//   seg.init(cfg, /*target_fps=*/10);
//   seg.start([&](cv::Mat& bgr, int64_t& stamp) {
//       auto cf = rx.color().GetData();
//       if (!cf || cf->empty()) return false;
//       stamp = cf->stamp_ns;
//       cv::Mat(cf->height, cf->width, CV_8UC3,
//               (void*)cf->data.data(), cf->stride_bytes).copyTo(bgr);
//       return true;
//   });
//   ... consume seg.result_buf ...
class SegInference {
public:
    // Fetch the latest camera frame. Return false if none; else fill `bgr`
    // (an owning copy — the source's buffer may be recycled) + `stamp_ns`.
    using FrameSource = std::function<bool(cv::Mat& bgr, int64_t& stamp_ns)>;

    SegInference() = default;
    ~SegInference();

    SegInference(const SegInference&) = delete;
    SegInference& operator=(const SegInference&) = delete;

    // Builds/loads the engine. target_fps caps the worker's inference rate.
    bool init(const YoloSegConfig& cfg, double target_fps = 10.0);

    bool start(FrameSource source);
    void stop();

    bool initialized() const { return engine_.initialized(); }

    // ── output: latest segmentation result (empty until the first frame) ──
    DataBuffer<SegResult> result_buf;

private:
    void run();

    YoloSegEngine engine_;
    FrameSource   source_;
    double        period_ms_  = 100.0;   // 1000 / target_fps
    int64_t       last_stamp_ = -1;

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
