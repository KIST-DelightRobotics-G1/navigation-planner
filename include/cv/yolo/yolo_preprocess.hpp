#pragma once

#include <opencv2/core.hpp>

namespace kist {

// Maps original-image pixels to the letterboxed model-input pixels:
//   input = original * scale + (pad_x, pad_y)
// Shared by every YOLO task (detection / segmentation / pose) — the network
// input format is identical, so this transform is not seg-specific.
struct LetterboxTransform {
    float scale   = 1.0f;   // original -> input (aspect-preserving)
    int   pad_x   = 0;      // left/top letterbox padding, input px
    int   pad_y   = 0;
    int   input_w = 0;
    int   input_h = 0;
};

// Scratch buffers reused across frames so the hot path allocates nothing
// (per-frame allocation of the ~11 MB blob caused preprocess-time spikes).
struct YoloPreprocessScratch {
    cv::Mat resized, canvas, blob;
};

// Letterbox `bgr` into an input_w x input_h canvas (aspect-preserving, 114 pad),
// convert BGR->RGB, scale to [0,1], and pack HWC->CHW into `dst` (must hold
// 3*input_h*input_w floats). Fills `lb` with the transform applied. This is the
// generic YOLO input preprocessing — reused across detection / seg / pose.
void yolo_preprocess(const cv::Mat& bgr, int input_w, int input_h,
                     float* dst, LetterboxTransform& lb,
                     YoloPreprocessScratch& scratch);

} // namespace kist
