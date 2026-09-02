#pragma once

#include "cv/yolo/instance_segmentation/yolo_inst_seg_postprocess.hpp"
#include "cv/yolo/instance_segmentation/yolo_inst_seg_frame.hpp"
#include "cv/yolo/yolo_inference.hpp"
#include "cv/yolo/yolo_pipeline.hpp"
#include "cv/yolo/yolo_preprocess.hpp"
#include "cv/yolo/yolo_timings.hpp"
#include "tensorrt/InferenceEngine.h"   // TPinnedVector

#include <opencv2/core.hpp>

#include <atomic>
#include <cstdint>
#include <string>

namespace kist {

struct YoloInstSegConfig {
    std::string onnx_path = "models/yolo26l-seg.onnx";

    float score_threshold = 0.25f;   // drop detections below this confidence

    // Tensor names (ultralytics YOLO26-seg export). Shapes are read from the
    // engine at init(), not hardcoded.
    std::string input_name = "images";   // [1, 3, H, W]
    std::string det_name   = "output0";  // [1, num_det, stride]  (NMS-free)
    std::string proto_name = "output1";  // [1, num_masks, ph, pw] (mask protos)
};

// YOLO26 instance-seg Module (no thread). Composes the three stages —
// yolo_preprocess (generic) -> YoloInference (generic) -> YoloInstSegPostprocess
// (cuBLAS proto GEMM) — and owns the pinned I/O buffers between them. The
// YoloPipeline worker thread drives it from the camera buffer; the static-image
// test runner drives it directly.
class YoloInstSegEngine {
public:
    // Pipeline plumbing (YoloPipeline<Engine> reads these).
    using Config = YoloInstSegConfig;
    using Result = InstSegFrame;

    YoloInstSegEngine() = default;

    bool init(const Config& cfg);

    // Runs one frame. stamp_ns is copied into the result. Boxes are in original-
    // image pixels; masks are at proto resolution (see InstSegFrame).
    InstSegFrame infer(const cv::Mat& bgr, int64_t stamp_ns = 0);

    bool initialized() const { return initialized_; }
    YoloStageTimings timings() const {
        return { pre_ms_.load(std::memory_order_relaxed),
                 inf_ms_.load(std::memory_order_relaxed),
                 post_ms_.load(std::memory_order_relaxed) };
    }

private:
    Config                 cfg_;
    YoloInference          infer_;
    YoloPreprocessScratch  pre_scratch_;
    YoloInstSegPostprocess post_;

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

// Ready-to-use pipeline for this task. Hides the YoloPipeline<Engine> template
// (and its generic Result slot) from callers — a consumer just sees a worker it
// start/stop()s whose result() is a DataBuffer<InstSegFrame>.
using YoloInstPipeline = YoloPipeline<YoloInstSegEngine>;

} // namespace kist
