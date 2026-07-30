#pragma once

#include <opencv2/core.hpp>

#include <cstdint>

namespace kist {

// Output of YoloSemSegEngine for one frame: a dense per-pixel class-id map. The
// semantic head emits an already-argmaxed map at the letterboxed model-input
// resolution (CV_8U, one class id per pixel), so there are no instances/masks —
// just the map plus the letterbox transform that maps an original-image pixel
// into it (for depth fusion / overlay). No upsample here: consumers sample only
// the pixels they need.
struct SemSegResult {
    int64_t stamp_ns = 0;              // carried through from the input frame
    int     width = 0, height = 0;     // original-image size

    // Class-id map at the letterboxed model-input resolution (CV_8U). Owning
    // copy — valid after the engine reuses its scratch on the next frame.
    cv::Mat class_map;                 // [input_height, input_width], class id per px

    int   input_width = 0, input_height = 0;  // model input (letterboxed) size
    float scale = 1.0f;                        // original -> letterboxed input
    int   pad_x = 0, pad_y = 0;                // letterbox padding, input px

    // Map an original-image pixel into class-map space (== letterboxed input px).
    cv::Point2f orig_to_map(float ox, float oy) const {
        return { ox * scale + pad_x, oy * scale + pad_y };
    }

    // Class id at an original-image pixel (-1 if it falls outside the map).
    int class_at(float ox, float oy) const {
        const int mx = int(ox * scale + pad_x), my = int(oy * scale + pad_y);
        if (class_map.empty() || mx < 0 || my < 0 ||
            mx >= class_map.cols || my >= class_map.rows)
            return -1;
        return class_map.at<uint8_t>(my, mx);
    }
};

} // namespace kist
