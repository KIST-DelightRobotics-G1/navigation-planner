#pragma once

#include "common/data_buffer.hpp"
#include "cv/yolo/segmentation/seg_result.hpp"
#include "cv/yolo/segmentation/yolo_seg_engine.hpp"

#include <opencv2/core.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>

namespace kist {

// Config for the whole CV inference subsystem. Bundles each model's config so a
// runner passes one struct. (Just segmentation today; detection/pose join here.)
struct CvInferenceConfig {
    YoloSegConfig seg;
    double        seg_target_fps = 30.0;   // 0 = uncapped
};

// Top-level CV inference subsystem and its worker Thread — the single entry point
// a runner/orchestrator touches. Owns the YOLO engine(s) and a worker loop that
// pulls the latest frame at a capped rate (latest-wins, so it never backs up
// behind a faster camera), runs inference, and publishes results. A runner only
// start()/stop()s it.
//
// Frames arrive via an injected source, so cv/ stays decoupled from any camera
// backend (builds without ext-sensor-io): the wiring layer supplies a lambda
// that reads e.g. RealsenseReceiver.color() and hands over an owning BGR copy.
//
// Usage (a runner's main):
//   CvInference cvi;
//   cvi.start(cfg, [&](cv::Mat& bgr, int64_t& stamp){ /* fill from camera */ });
//   ... read cvi.seg_result() ...
//   cvi.stop();
class CvInference {
public:
    // Fetch the latest camera frame. Return false if none; else fill `bgr`
    // (an owning copy — the source's buffer may be recycled) + `stamp_ns`.
    using FrameSource = std::function<bool(cv::Mat& bgr, int64_t& stamp_ns)>;

    CvInference() = default;
    ~CvInference() { stop(); }

    CvInference(const CvInference&) = delete;
    CvInference& operator=(const CvInference&) = delete;

    // Build the model(s) and start the worker off `source`. False on failure.
    bool start(const CvInferenceConfig& cfg, FrameSource source);
    void stop();
    bool running() const { return running_; }

    // ── segmentation outputs / telemetry ──
    DataBuffer<SegResult>& seg_result() { return seg_result_; }
    uint64_t   seg_frames_processed() const { return processed_.load(std::memory_order_relaxed); }
    SegTimings seg_timings() const { return seg_engine_.timings(); }

private:
    void run();

    YoloSegEngine seg_engine_;
    DataBuffer<SegResult> seg_result_;

    FrameSource source_;
    double      period_ms_  = 0.0;    // 1000 / target_fps; 0 = uncapped
    int64_t     last_stamp_ = -1;

    std::thread           thread_;
    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> processed_{0};
};

} // namespace kist
