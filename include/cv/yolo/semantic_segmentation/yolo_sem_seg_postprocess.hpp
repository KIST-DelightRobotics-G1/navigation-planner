#pragma once

#include "cv/yolo/semantic_segmentation/yolo_sem_seg_result.hpp"
#include "cv/yolo/yolo_preprocess.hpp"   // LetterboxTransform

#include <cstdint>

namespace kist {

// YOLO semantic-seg postprocess: the head already emits an argmaxed UINT8 class
// map at the letterboxed model-input resolution, so there is nothing to decode —
// wrap the map into the result (owning copy) and record the letterbox transform
// so consumers can map original px -> class-map px. The caller sets out.stamp_ns.
void yolo_sem_seg_postprocess(const uint8_t* class_map_host, int map_w, int map_h,
                              const LetterboxTransform& lb, int orig_w, int orig_h,
                              SemSegResult& out);

} // namespace kist
