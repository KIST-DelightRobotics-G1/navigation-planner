#include "cv/cv_inference.hpp"

#include <pthread.h>

#include <chrono>
#include <utility>

namespace kist {

bool CvInference::start(const CvInferenceConfig& cfg, FrameSource source) {
    if (running_) return true;
    if (!seg_engine_.init(cfg.seg)) return false;
    period_ms_ = (cfg.seg_target_fps > 0.0) ? (1000.0 / cfg.seg_target_fps) : 0.0;
    source_    = std::move(source);
    running_   = true;
    thread_    = std::thread(&CvInference::run, this);
    return true;
}

void CvInference::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void CvInference::run() {
    pthread_setname_np(pthread_self(), "cv-infer");
    using clock = std::chrono::steady_clock;

    while (running_) {
        const auto t0 = clock::now();

        cv::Mat bgr;
        int64_t stamp = 0;
        // Latest-wins: process only a new frame, skip stale/duplicate stamps.
        if (source_ && source_(bgr, stamp) && stamp != last_stamp_ && !bgr.empty()) {
            last_stamp_ = stamp;
            seg_result_.SetData(seg_engine_.infer(bgr, stamp));
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
