#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <vector>

namespace kist {

// One segmented instance from YoloInstSegEngine.
struct InstSegDetection {
    cv::Rect box;        // bounding box in original-image pixels
    float    score = 0;  // confidence
    int      class_id = -1;
    // Instance mask at PROTO resolution (InstSegResult.mask_width x mask_height),
    // CV_8U (0/255), zero outside the instance. Kept at the network's prototype
    // resolution — not upsampled to the frame — so postprocess stays O(proto),
    // independent of object size/count (matches Ultralytics process_mask with
    // upsample=False). Aligned to the letterboxed model input; use
    // InstSegResult::orig_to_mask() to sample it at an original-image pixel.
    cv::Mat  mask;
};

// Output of YoloInstSegEngine for one frame. Masks live at prototype resolution;
// the letterbox transform below maps an original-image pixel into mask space so
// any consumer (viewer overlay, depth fusion) can sample without the engine
// materializing a full-resolution mask per instance.
struct InstSegResult {
    int64_t stamp_ns = 0;              // carried through from the input frame
    int     width = 0, height = 0;     // original-image size

    // Proto-mask geometry + letterbox transform. An original pixel maps to mask
    // space as: mask = (orig * scale + pad) * (mask_dim / input_dim).
    int   mask_width = 0,  mask_height = 0;   // prototype mask resolution
    int   input_width = 0, input_height = 0;  // model input (letterboxed) size
    float scale = 1.0f;                        // original -> letterboxed input
    int   pad_x = 0, pad_y = 0;                // letterbox padding, input px

    std::vector<InstSegDetection> detections;

    // Map an original-image pixel to its location in the proto-resolution mask.
    cv::Point2f orig_to_mask(float ox, float oy) const {
        const float ix = ox * scale + pad_x, iy = oy * scale + pad_y;
        return { ix * mask_width  / (input_width  ? input_width  : 1),
                 iy * mask_height / (input_height ? input_height : 1) };
    }
};

} // namespace kist
