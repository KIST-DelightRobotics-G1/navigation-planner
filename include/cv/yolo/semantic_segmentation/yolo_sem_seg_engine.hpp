#pragma once

#include "cv/yolo/semantic_segmentation/yolo_sem_seg_result.hpp"
#include "cv/yolo/yolo_inference.hpp"
#include "cv/yolo/yolo_preprocess.hpp"
#include "cv/yolo/yolo_timings.hpp"
#include "tensorrt/InferenceEngine.h"   // TPinnedVector

#include <opencv2/core.hpp>

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace kist {

struct YoloSemSegConfig {
    std::string onnx_path = "models/yolo26l-sem.onnx";

    // Tensor names (ultralytics YOLO26-sem export). Shapes read at init().
    std::string input_name  = "images";    // [1, 3, H, W] (FLOAT)
    std::string output_name = "output0";   // [1, H, W]    (UINT8 class-id map)
};

// YOLO26 semantic-seg Module (no thread). Composes yolo_preprocess (generic) ->
// YoloInference (generic) -> yolo_sem_seg_postprocess (trivial: the head already
// emits an argmaxed class map). Owns the pinned input + the UINT8 class-map
// scratch. The YoloPipeline worker thread drives it; the static-image test runner
// drives it directly.
class YoloSemSegEngine {
public:
    // Pipeline plumbing (YoloPipeline<Engine> reads these).
    using Config = YoloSemSegConfig;
    using Result = SemSegResult;

    YoloSemSegEngine() = default;

    bool init(const Config& cfg);

    SemSegResult infer(const cv::Mat& bgr, int64_t stamp_ns = 0);

    bool initialized() const { return initialized_; }
    YoloStageTimings timings() const {
        return { pre_ms_.load(std::memory_order_relaxed),
                 inf_ms_.load(std::memory_order_relaxed),
                 post_ms_.load(std::memory_order_relaxed) };
    }

private:
    Config                cfg_;
    YoloInference         infer_;
    YoloPreprocessScratch pre_scratch_;

    int input_w_ = 0, input_h_ = 0;   // model input (from engine)
    int out_w_ = 0, out_h_ = 0;       // output class map size

    TPinnedVector<float>  input_buf_;   // pinned FLOAT input
    std::vector<uint8_t>  class_buf_;   // host UINT8 class-id map (D2H target)

    std::atomic<double> pre_ms_{0.0}, inf_ms_{0.0}, post_ms_{0.0};

    bool initialized_ = false;
};

} // namespace kist
