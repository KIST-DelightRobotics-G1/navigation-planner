#include "segmentation/segment_inference.hpp"

#include <pthread.h>

#include <chrono>
#include <utility>

namespace kist {

SegInference::~SegInference() { stop(); }

bool SegInference::init(const YoloSegConfig& cfg, double target_fps) {
    period_ms_ = (target_fps > 0.0) ? (1000.0 / target_fps) : 0.0;
    return engine_.init(cfg);
}

bool SegInference::start(FrameSource source) {
    if (running_) return true;
    if (!engine_.initialized()) return false;
    source_  = std::move(source);
    running_ = true;
    thread_  = std::thread(&SegInference::run, this);
    return true;
}

void SegInference::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void SegInference::run() {
    pthread_setname_np(pthread_self(), "seg-infer");
    using clock = std::chrono::steady_clock;

    while (running_) {
        const auto t0 = clock::now();

        cv::Mat bgr;
        int64_t stamp = 0;
        // Latest-wins: process only a new frame, skip stale/duplicate stamps.
        if (source_ && source_(bgr, stamp) && stamp != last_stamp_ && !bgr.empty()) {
            last_stamp_ = stamp;
            result_buf.SetData(engine_.infer(bgr, stamp));
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

} // namespace kist
