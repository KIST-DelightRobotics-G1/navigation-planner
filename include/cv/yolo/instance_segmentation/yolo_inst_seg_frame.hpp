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
    // Instance mask at PROTO resolution (InstSegFrame.mask_width x mask_height),
    // CV_8U (0/255), zero outside the instance. Kept at the network's prototype
    // resolution — not upsampled to the frame — so postprocess stays O(proto),
    // independent of object size/count (matches Ultralytics process_mask with
    // upsample=False). Aligned to the letterboxed model input; use
    // InstSegFrame::orig_to_mask() to sample it at an original-image pixel.
    cv::Mat  mask;
};

// Output of YoloInstSegEngine for one frame. Masks live at prototype resolution;
// the letterbox transform below maps an original-image pixel into mask space so
// any consumer (viewer overlay, depth fusion) can sample without the engine
// materializing a full-resolution mask per instance.
struct InstSegFrame {
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

    // Class id of the highest-scoring instance whose mask covers an original-
    // image pixel, or -1 if none (background / undetected). Same signature as
    // SemSegFrame::class_at, so depth fusion samples either the same way — but
    // here only detected objects carry a label (the rest stay unlabeled).
    int class_at(float ox, float oy) const {
        int   best = -1;
        float best_score = -1.0f;
        const cv::Point p{int(ox), int(oy)};
        for (const auto& d : detections) {
            if (d.score <= best_score || !d.box.contains(p)) continue;  // can't win / outside box
            const cv::Point2f m = orig_to_mask(ox, oy);
            const int mx = int(m.x), my = int(m.y);
            if (mx < 0 || my < 0 || mx >= d.mask.cols || my >= d.mask.rows) continue;
            if (d.mask.at<uint8_t>(my, mx) > 0) { best = d.class_id; best_score = d.score; }
        }
        return best;
    }

    // Index of the highest-scoring instance covering an original-image pixel, or
    // -1. This is a PER-FRAME detection index (order changes across frames), not
    // a stable identity — usable as a same-frame seed to split touching objects,
    // not to remember an object over time (that needs tracking).
    int instance_at(float ox, float oy) const {
        int best = -1; float best_score = -1.0f;
        const cv::Point p{int(ox), int(oy)};
        for (size_t i = 0; i < detections.size(); ++i) {
            const auto& d = detections[i];
            if (d.score <= best_score || !d.box.contains(p)) continue;
            const cv::Point2f m = orig_to_mask(ox, oy);
            const int mx = int(m.x), my = int(m.y);
            if (mx < 0 || my < 0 || mx >= d.mask.cols || my >= d.mask.rows) continue;
            if (d.mask.at<uint8_t>(my, mx) > 0) { best = int(i); best_score = d.score; }
        }
        return best;
    }
};

} // namespace kist
