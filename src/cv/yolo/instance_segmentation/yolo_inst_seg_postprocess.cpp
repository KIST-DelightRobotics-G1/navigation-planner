#include "cv/yolo/instance_segmentation/yolo_inst_seg_postprocess.hpp"

#include <opencv2/core.hpp>

#include <cublas_v2.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace kist {

namespace {
// Cap on instances handled per frame (sizes the device/host mask scratch). Well
// above any realistic scene; extra detections past this are dropped.
constexpr int kMaxInstances = 64;
}

struct YoloInstSegPostprocess::Impl {
    cublasHandle_t handle = nullptr;
    cudaStream_t   stream = nullptr;

    int det_count = 0, det_stride = 0;
    int proto_c = 0, proto_h = 0, proto_w = 0;
    int area = 0;          // proto_h * proto_w
    int max_n = 0;         // min(det_count, kMaxInstances)

    float* d_coeffs = nullptr;   // device [max_n, proto_c]
    float* d_masks  = nullptr;   // device [max_n, area]
    float* h_coeffs = nullptr;   // pinned [max_n, proto_c]
    float* h_masks  = nullptr;   // pinned [max_n, area]

    ~Impl() {
        if (d_coeffs) cudaFree(d_coeffs);
        if (d_masks)  cudaFree(d_masks);
        if (h_coeffs) cudaFreeHost(h_coeffs);
        if (h_masks)  cudaFreeHost(h_masks);
        if (handle)   cublasDestroy(handle);
    }
};

YoloInstSegPostprocess::YoloInstSegPostprocess() : impl_(std::make_unique<Impl>()) {}
YoloInstSegPostprocess::~YoloInstSegPostprocess() = default;

bool YoloInstSegPostprocess::init(int det_count, int det_stride,
                                  int proto_c, int proto_h, int proto_w,
                                  cudaStream_t stream) {
    auto& im = *impl_;
    im.det_count = det_count; im.det_stride = det_stride;
    im.proto_c = proto_c; im.proto_h = proto_h; im.proto_w = proto_w;
    im.area  = proto_h * proto_w;
    im.max_n = std::min(det_count, kMaxInstances);
    im.stream = stream;

    if (cublasCreate(&im.handle) != CUBLAS_STATUS_SUCCESS) {
        std::fprintf(stderr, "[YoloInstSegPostprocess] cublasCreate failed\n");
        return false;
    }
    cublasSetStream(im.handle, stream);

    const size_t coeff_n = size_t(im.max_n) * im.proto_c;
    const size_t mask_n  = size_t(im.max_n) * im.area;
    if (cudaMalloc(reinterpret_cast<void**>(&im.d_coeffs), coeff_n * sizeof(float)) != cudaSuccess ||
        cudaMalloc(reinterpret_cast<void**>(&im.d_masks),  mask_n  * sizeof(float)) != cudaSuccess ||
        cudaMallocHost(reinterpret_cast<void**>(&im.h_coeffs), coeff_n * sizeof(float)) != cudaSuccess ||
        cudaMallocHost(reinterpret_cast<void**>(&im.h_masks),  mask_n  * sizeof(float)) != cudaSuccess) {
        std::fprintf(stderr, "[YoloInstSegPostprocess] scratch alloc failed\n");
        return false;
    }
    return true;
}

void YoloInstSegPostprocess::run(const float* det_host, const void* proto_device,
                                 const LetterboxTransform& lb, int orig_w, int orig_h,
                                 float score_threshold, InstSegResult& out) {
    auto& im = *impl_;

    out.width  = orig_w;       out.height = orig_h;
    out.mask_width  = im.proto_w; out.mask_height = im.proto_h;
    out.input_width = lb.input_w; out.input_height = lb.input_h;
    out.scale = lb.scale; out.pad_x = lb.pad_x; out.pad_y = lb.pad_y;

    const float sx = im.proto_w / float(lb.input_w), sy = im.proto_h / float(lb.input_h);
    const float scale = lb.scale;
    const int   px = lb.pad_x, py = lb.pad_y;

    // Pass 1 (CPU): keep detections above threshold; gather their coeffs into the
    // pinned staging buffer + record box/pbox. Boxes are in original px; pboxes
    // are the same boxes in proto coords (proto is aligned to the letterboxed input).
    struct Keep { cv::Rect box, pbox; float score; int class_id; };
    std::vector<Keep> keep;
    keep.reserve(im.max_n);
    int n = 0;

    for (int i = 0; i < im.det_count && n < im.max_n; ++i) {
        const float* d = det_host + size_t(i) * im.det_stride;
        const float score = d[4];
        if (score < score_threshold) continue;

        float x1 = (d[0] - px) / scale, y1 = (d[1] - py) / scale;
        float x2 = (d[2] - px) / scale, y2 = (d[3] - py) / scale;
        x1 = std::clamp(x1, 0.f, float(orig_w));  y1 = std::clamp(y1, 0.f, float(orig_h));
        x2 = std::clamp(x2, 0.f, float(orig_w));  y2 = std::clamp(y2, 0.f, float(orig_h));
        cv::Rect box(int(x1), int(y1), int(x2 - x1), int(y2 - y1));
        box &= cv::Rect(0, 0, orig_w, orig_h);
        if (box.width <= 0 || box.height <= 0) continue;

        cv::Rect pbox(int(std::floor(d[0] * sx)), int(std::floor(d[1] * sy)), 0, 0);
        pbox.width  = int(std::ceil(d[2] * sx)) - pbox.x;
        pbox.height = int(std::ceil(d[3] * sy)) - pbox.y;
        pbox &= cv::Rect(0, 0, im.proto_w, im.proto_h);
        if (pbox.width <= 0 || pbox.height <= 0) continue;

        std::copy(d + 6, d + 6 + im.proto_c, im.h_coeffs + size_t(n) * im.proto_c);
        keep.push_back({box, pbox, score, int(d[5])});
        ++n;
    }
    if (n == 0) return;

    // Pass 2 (GPU): one batched GEMM on the inference stream —
    //   masks[n, area] = coeffs[n, C] x proto[C, area]
    // proto stays on the device; only coeffs (tiny) go up and the mask logits
    // come back. cuBLAS is column-major, so row-major C=A*B is issued in the
    // equivalent (area, n, C) form.
    const float alpha = 1.0f, beta = 0.0f;
    cudaMemcpyAsync(im.d_coeffs, im.h_coeffs, size_t(n) * im.proto_c * sizeof(float),
                    cudaMemcpyHostToDevice, im.stream);
    cublasSgemm(im.handle, CUBLAS_OP_N, CUBLAS_OP_N,
                im.area, n, im.proto_c,
                &alpha,
                static_cast<const float*>(proto_device), im.area,   // B [C, area]
                im.d_coeffs, im.proto_c,                            // A [n, C]
                &beta,
                im.d_masks, im.area);                               // C [n, area]
    cudaMemcpyAsync(im.h_masks, im.d_masks, size_t(n) * im.area * sizeof(float),
                    cudaMemcpyDeviceToHost, im.stream);
    cudaStreamSynchronize(im.stream);

    // Pass 3 (CPU): each mask finished at proto resolution — crop to the box and
    // threshold the raw logit at 0. No upsample, no full-frame alloc.
    for (int k = 0; k < n; ++k) {
        const cv::Mat m(im.proto_h, im.proto_w, CV_32F, im.h_masks + size_t(k) * im.area);

        cv::Mat mask = cv::Mat::zeros(im.proto_h, im.proto_w, CV_8U);
        cv::Mat bin = m(keep[k].pbox) > 0.0f;   // logit>0 -> CV_8U 0/255
        bin.copyTo(mask(keep[k].pbox));

        InstSegDetection dobj;
        dobj.box      = keep[k].box;
        dobj.score    = keep[k].score;
        dobj.class_id = keep[k].class_id;
        dobj.mask     = std::move(mask);
        out.detections.push_back(std::move(dobj));
    }
}

} // namespace kist
