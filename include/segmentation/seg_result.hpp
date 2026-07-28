#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <vector>

namespace kist {

// One segmented instance from YoloSegEngine.
struct SegDetection {
    cv::Rect box;        // bounding box in original-image pixels
    float    score = 0;  // confidence
    int      class_id = -1;
    cv::Mat  mask;       // CV_8U (0/255), original-image size, non-zero inside the instance
};

// Output of YoloSegEngine for one frame.
struct SegResult {
    int64_t stamp_ns = 0;              // carried through from the input frame
    int     width = 0, height = 0;     // original-image size
    std::vector<SegDetection> detections;
};

} // namespace kist
