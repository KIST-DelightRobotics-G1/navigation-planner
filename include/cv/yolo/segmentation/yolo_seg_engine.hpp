#pragma once

#include "cv/yolo/segmentation/seg_result.hpp"
#include "cv/yolo/segmentation/yolo_seg_postprocess.hpp"
#include "cv/yolo/yolo_inference.hpp"
#include "cv/yolo/yolo_preprocess.hpp"
#include "tensorrt/InferenceEngine.h"   // TPinnedVector

#include <opencv2/core.hpp>

#include <atomic>
#include <cstdint>
#include <string>

namespace kist {

// Per-stage wall-clock cost of the last infer() call (ms). Diagnostic only —
// lets a runner see whether a frame is preprocess-, GPU-, or postprocess-bound.
// `infer` covers the whole H2D + enqueue + D2H block up to the stream sync.
struct SegTimings {
    double preprocess_ms  = 0.0;
    double infer_ms       = 0.0;
    double postprocess_ms = 0.0;
};

struct YoloSegConfig {
    std::string onnx_path = "models/yolo26l-seg.onnx";

    float score_threshold = 0.25f;   // drop detections below this confidence

    // Tensor names (ultralytics YOLO26-seg export). Shapes are read from the
    // engine at init(), not hardcoded.
    std::string input_name = "images";   // [1, 3, H, W]
    std::string det_name   = "output0";  // [1, num_det, stride]  (NMS-free)
    std::string proto_name = "output1";  // [1, num_masks, ph, pw] (mask protos)
};

// YOLO26-seg inference Module (no thread). A thin orchestrator that composes the
// three stages — yolo_preprocess (generic) -> YoloInference (generic) ->
// yolo_seg_postprocess (seg-specific) — and owns the pinned I/O buffers between
// them. The CvInference worker thread drives this from the camera buffer; the
// static-image test runner drives it directly.
class YoloSegEngine {
public:
    YoloSegEngine() = default;

    // Builds/loads the engine. Returns false (with a logged reason) on failure.
    bool init(const YoloSegConfig& cfg);

    // Runs one frame. stamp_ns is copied into the result. Boxes are in original-
    // image pixels; masks are at proto resolution (see SegResult). Returns an
    // empty result if not initialized.
    SegResult infer(const cv::Mat& bgr, int64_t stamp_ns = 0);

    bool initialized() const { return initialized_; }

    // Per-stage cost of the most recent infer() (ms). Cheap to poll from another
    // thread; values are a single-frame snapshot, not smoothed.
    SegTimings timings() const {
        return { pre_ms_.load(std::memory_order_relaxed),
                 inf_ms_.load(std::memory_order_relaxed),
                 post_ms_.load(std::memory_order_relaxed) };
    }

private:
    YoloSegConfig         cfg_;
    YoloInference         infer_;
    YoloPreprocessScratch pre_scratch_;
    YoloSegPostprocess    post_;

    int input_w_ = 0, input_h_ = 0;              // model input (from engine)
    int det_count_ = 0, det_stride_ = 0;         // output0: [det_count_, det_stride_]
    int proto_c_ = 0, proto_h_ = 0, proto_w_ = 0;// output1: [c, h, w]

    // Input + detection tensors round-trip through the host; the prototype
    // tensor stays on the GPU (postprocess reads it there via cuBLAS).
    TPinnedVector<float> input_buf_;
    TPinnedVector<float> det_buf_;

    std::atomic<double> pre_ms_{0.0}, inf_ms_{0.0}, post_ms_{0.0};

    bool initialized_ = false;
};

} // namespace kist
