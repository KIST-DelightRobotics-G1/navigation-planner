#include "cv/yolo/semantic_segmentation/yolo_sem_seg_postprocess.hpp"

#include <opencv2/core.hpp>

namespace kist {

void yolo_sem_seg_postprocess(const uint8_t* class_map_host, int map_w, int map_h,
                              const LetterboxTransform& lb, int orig_w, int orig_h,
                              SemSegFrame& out) {
    out.width  = orig_w;      out.height = orig_h;
    out.input_width = lb.input_w; out.input_height = lb.input_h;
    out.scale = lb.scale; out.pad_x = lb.pad_x; out.pad_y = lb.pad_y;

    // The map is at the model-input resolution (== letterboxed input); wrap the
    // D2H'd scratch and clone into the result so it survives the next frame.
    const cv::Mat wrapped(map_h, map_w, CV_8U, const_cast<uint8_t*>(class_map_host));
    out.class_map = wrapped.clone();
}

} // namespace kist
