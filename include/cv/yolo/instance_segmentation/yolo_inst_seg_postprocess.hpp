#pragma once

#include "cv/yolo/instance_segmentation/yolo_inst_seg_frame.hpp"
#include "cv/yolo/yolo_preprocess.hpp"   // LetterboxTransform

#include <memory>

// Forward-declare the CUDA stream type (avoid pulling cuda headers here).
typedef struct CUstream_st* cudaStream_t;

namespace kist {

// NMS-free YOLO instance-seg postprocess: decode raw model outputs into an
// InstSegFrame, with the coeff x proto GEMM run on the GPU (cuBLAS). The
// prototype tensor stays on the device (no device->host copy); only the small
// detection tensor is on the host (for the score filter) and only the resulting
// per-instance mask logits come back. Masks are kept at PROTO resolution
// (Ultralytics process_mask, upsample=False): crop to the box and threshold the
// logit at 0 (sigmoid(x)>0.5 <=> x>0). Owns the cuBLAS handle + device/host
// scratch, so it is a small stateful object rather than a free function.
class YoloInstSegPostprocess {
public:
    YoloInstSegPostprocess();
    ~YoloInstSegPostprocess();

    YoloInstSegPostprocess(const YoloInstSegPostprocess&) = delete;
    YoloInstSegPostprocess& operator=(const YoloInstSegPostprocess&) = delete;

    // Allocate the cuBLAS handle + scratch for these output shapes. `stream` is
    // the inference stream (the GEMM runs on it after the outputs are ready).
    bool init(int det_count, int det_stride,
              int proto_c, int proto_h, int proto_w, cudaStream_t stream);

    // det_host   : host   [det_count, det_stride] rows [x1,y1,x2,y2,score,cls,m0..]
    // proto_device: device [proto_c, proto_h*proto_w] mask prototypes (stays on GPU)
    // Fills out.detections + the mask-geometry/letterbox fields; the caller sets
    // out.stamp_ns.
    void run(const float* det_host, const void* proto_device,
             const LetterboxTransform& lb, int orig_w, int orig_h,
             float score_threshold, int mask_erode_px, InstSegFrame& out);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace kist
