#pragma once

#include "segmentation/seg_result.hpp"
#include "tensorrt/InferenceEngine.h"

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>

// Forward-declare the CUDA stream type (avoid pulling cuda headers here).
typedef struct CUstream_st* cudaStream_t;

namespace kist {

struct YoloSegConfig {
    std::string onnx_path = "models/yolo26l-seg.onnx";

    float score_threshold = 0.25f;   // drop detections below this confidence
    float mask_threshold  = 0.50f;   // proto-mask sigmoid cutoff -> binary

    // Tensor names (ultralytics YOLO26-seg export). The shapes are read from
    // the engine at init(), not hardcoded.
    std::string input_name = "images";   // [1, 3, H, W]
    std::string det_name   = "output0";  // [1, num_det, stride]  (NMS-free)
    std::string proto_name = "output1";  // [1, num_masks, ph, pw] (mask protos)
};

// YOLO26-seg inference Module (no thread, no I/O beyond the model). Loads the
// ONNX, builds/caches a FP16 TensorRT engine (via the vendored TRTInferenceEngine),
// and runs one BGR frame per infer() call: letterbox preprocess -> TRT infer ->
// NMS-free postprocess (YOLO26's head already emits top-N detections, so there
// is no anchor decode / NMS to do — just filter, un-letterbox boxes, and build
// masks from the proto coefficients).
//
// A Thread (SegInference) drives this from the camera buffer; the static-image
// test runner drives it directly.
class YoloSegEngine {
public:
    YoloSegEngine() = default;
    ~YoloSegEngine();

    YoloSegEngine(const YoloSegEngine&) = delete;
    YoloSegEngine& operator=(const YoloSegEngine&) = delete;

    // Builds/loads the engine. Returns false (with a logged reason) on failure.
    bool init(const YoloSegConfig& cfg);

    // Runs one frame. stamp_ns is copied into the result. Boxes/masks are in
    // original-image pixels. Returns an empty result if not initialized.
    SegResult infer(const cv::Mat& bgr, int64_t stamp_ns = 0);

    bool initialized() const { return initialized_; }

private:
    YoloSegConfig      cfg_;
    TRTInferenceEngine engine_;
    cudaStream_t       stream_ = nullptr;

    int input_w_ = 0, input_h_ = 0;              // model input (from engine)
    int det_count_ = 0, det_stride_ = 0;         // output0: [det_count_, det_stride_]
    int proto_c_ = 0, proto_h_ = 0, proto_w_ = 0;// output1: [c, h, w]

    TPinnedVector<float> input_buf_;
    TPinnedVector<float> det_buf_;
    TPinnedVector<float> proto_buf_;

    bool initialized_ = false;
};

} // namespace kist
