#pragma once

#include "common/data_buffer.hpp"
#include "cv/yolo/yolo_timings.hpp"

#include <opencv2/core.hpp>

#include <pthread.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>
#include <utility>

namespace kist {

// The YOLO inference pipeline and its worker Thread — the single entry point a
// runner/orchestrator touches, generic over the task engine. Owns the engine
// and a worker loop that pulls the latest frame at a capped rate (latest-wins,
// so it never backs up behind a faster camera), runs inference, and publishes
// results. A runner only start()/stop()s it.
//
// `Engine` is a passive Module (instance-seg or semantic-seg) exposing:
//     using Config; using Result;
//     bool init(const Config&);
//     Result infer(const cv::Mat& bgr, int64_t stamp_ns);
//     bool initialized() const;
//     YoloStageTimings timings() const;
// so the worker loop is written once here and instantiated per task:
//     using YoloInstPipeline = YoloPipeline<YoloInstSegEngine>;
//     using YoloSemPipeline  = YoloPipeline<YoloSemSegEngine>;
//
// Frames arrive via an injected source, so cv/ stays decoupled from any camera
// backend (builds without ext-sensor-io).
template <class Engine>
class YoloPipeline {
public:
    using Config = typename Engine::Config;
    using Result = typename Engine::Result;
    // Fetch the latest camera frame. Return false if none; else fill `bgr` (an
    // owning copy — the source's buffer may be recycled) + `stamp_ns`.
    using FrameSource = std::function<bool(cv::Mat& bgr, int64_t& stamp_ns)>;

    YoloPipeline() = default;
    ~YoloPipeline() { stop(); }

    YoloPipeline(const YoloPipeline&) = delete;
    YoloPipeline& operator=(const YoloPipeline&) = delete;

    // Build the model and start the worker off `source`. target_fps caps the
    // worker's inference rate (0 = uncapped). False on failure.
    bool start(const Config& cfg, double target_fps, FrameSource source) {
        if (running_) return true;
        if (!engine_.init(cfg)) return false;
        period_ms_ = (target_fps > 0.0) ? (1000.0 / target_fps) : 0.0;
        source_    = std::move(source);
        running_   = true;
        thread_    = std::thread(&YoloPipeline::run, this);
        return true;
    }

    void stop() {
        running_ = false;
        if (thread_.joinable())
            thread_.join();
    }

    bool running() const { return running_; }

    // ── output / telemetry ──
    DataBuffer<Result>& result() { return result_; }
    uint64_t         frames_processed() const { return processed_.load(std::memory_order_relaxed); }
    YoloStageTimings timings() const { return engine_.timings(); }

private:
    void run() {
        pthread_setname_np(pthread_self(), "yolo-pipe");
        using clock = std::chrono::steady_clock;

        while (running_) {
            const auto t0 = clock::now();

            cv::Mat bgr;
            int64_t stamp = 0;
            // Latest-wins: process only a new frame, skip stale/duplicate stamps.
            if (source_ && source_(bgr, stamp) && stamp != last_stamp_ && !bgr.empty()) {
                last_stamp_ = stamp;
                result_.SetData(engine_.infer(bgr, stamp));
                processed_.fetch_add(1, std::memory_order_relaxed);
            }

            // Pace to the target rate (drop the wait if inference already overran).
            if (period_ms_ > 0.0) {
                const auto period = std::chrono::duration_cast<clock::duration>(
                    std::chrono::duration<double, std::milli>(period_ms_));
                const auto elapsed = clock::now() - t0;
                if (elapsed < period)
                    std::this_thread::sleep_for(period - elapsed);
            }
        }
    }

    Engine             engine_;
    DataBuffer<Result> result_;

    FrameSource source_;
    double      period_ms_  = 0.0;    // 1000 / target_fps; 0 = uncapped
    int64_t     last_stamp_ = -1;

    std::thread           thread_;
    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> processed_{0};
};

} // namespace kist
