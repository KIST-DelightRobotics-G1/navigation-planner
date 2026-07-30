#pragma once

#include "cv/yolo/segmentation/seg_result.hpp"
#include "cv/yolo/yolo_preprocess.hpp"   // LetterboxTransform

namespace kist {

// NMS-free YOLO-seg postprocess: decode raw model outputs into a SegResult.
//   det   : [det_count, det_stride] rows [x1,y1,x2,y2,score,class,m0..m(C-1)]
//           in letterboxed-input pixels (YOLO26 head is NMS-free, top-N already)
//   proto : [proto_c, proto_h, proto_w] shared mask prototypes
// Masks are produced at PROTO resolution (Ultralytics process_mask,
// upsample=False): one batched coeff x proto GEMM, then per detection crop to
// the box and threshold the logit at 0 (sigmoid(x)>0.5 <=> x>0). `lb` maps
// original px into the letterboxed input; orig_w/orig_h size the output frame.
// Fills out.detections plus the mask-geometry/letterbox fields; the caller sets
// out.stamp_ns. See SegResult for the mask coordinate convention.
void yolo_seg_postprocess(const float* det, int det_count, int det_stride,
                          const float* proto, int proto_c, int proto_h, int proto_w,
                          const LetterboxTransform& lb, int orig_w, int orig_h,
                          float score_threshold, SegResult& out);

} // namespace kist
