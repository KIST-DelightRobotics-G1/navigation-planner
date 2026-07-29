#include "segmentation/yolo/yolo_seg_engine.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

namespace kist {

namespace {

// Read a tensor's flattened element count and per-dim sizes from the engine.
std::vector<int> tensor_shape(const TRTInferenceEngine& engine, const std::string& name) {
    std::vector<int64_t> shape;
    std::vector<int> out;
    if (engine.GetTensorShape(name, shape))
        for (auto d : shape) out.push_back(static_cast<int>(d));
    return out;
}

} // namespace

YoloSegEngine::~YoloSegEngine() {
    if (stream_) {
        cudaStreamDestroy(stream_);
        stream_ = nullptr;
    }
}

bool YoloSegEngine::init(const YoloSegConfig& cfg) {
    cfg_ = cfg;

    // Build (or load cached) a FP16 engine from the ONNX.
    Options opts;
    opts.precision = Precision::FP16;
    opts.deviceID  = 0;
    std::string trt_path;
    if (!ConvertONNXToTRT(opts, cfg_.onnx_path, trt_path)) {
        std::cerr << "[YoloSegEngine] ConvertONNXToTRT failed: " << cfg_.onnx_path << "\n";
        return false;
    }
    if (!engine_.Initialize(trt_path, 0) || !engine_.InitInputs()) {
        std::cerr << "[YoloSegEngine] engine init failed: " << trt_path << "\n";
        return false;
    }

    // Resolve shapes from the engine (don't hardcode).
    auto in_s    = tensor_shape(engine_, cfg_.input_name);   // [1, 3, H, W]
    auto det_s   = tensor_shape(engine_, cfg_.det_name);     // [1, N, S]
    auto proto_s = tensor_shape(engine_, cfg_.proto_name);   // [1, C, ph, pw]
    if (in_s.size() != 4 || det_s.size() != 3 || proto_s.size() != 4) {
        std::cerr << "[YoloSegEngine] unexpected tensor ranks (in=" << in_s.size()
                  << " det=" << det_s.size() << " proto=" << proto_s.size() << ")\n";
        return false;
    }
    input_h_    = in_s[2];    input_w_    = in_s[3];
    det_count_  = det_s[1];   det_stride_ = det_s[2];
    proto_c_    = proto_s[1]; proto_h_    = proto_s[2]; proto_w_ = proto_s[3];

    if (cudaStreamCreate(&stream_) != cudaSuccess) {
        std::cerr << "[YoloSegEngine] cudaStreamCreate failed\n";
        return false;
    }
    input_buf_.resize(static_cast<size_t>(3) * input_h_ * input_w_);
    det_buf_.resize(static_cast<size_t>(det_count_) * det_stride_);
    proto_buf_.resize(static_cast<size_t>(proto_c_) * proto_h_ * proto_w_);

    std::cout << "[YoloSegEngine] ready: input " << input_w_ << "x" << input_h_
              << ", det [" << det_count_ << "," << det_stride_ << "]"
              << ", proto [" << proto_c_ << "," << proto_h_ << "," << proto_w_ << "]\n";
    initialized_ = true;
    return true;
}

SegResult YoloSegEngine::infer(const cv::Mat& bgr, int64_t stamp_ns) {
    SegResult result;
    if (!initialized_ || bgr.empty()) return result;
    result.stamp_ns = stamp_ns;
    result.width    = bgr.cols;
    result.height   = bgr.rows;

    const int ow = bgr.cols, oh = bgr.rows;

    using clock = std::chrono::steady_clock;
    auto ms_since = [](clock::time_point t) {
        return std::chrono::duration<double, std::milli>(clock::now() - t).count();
    };
    const auto t_pre = clock::now();

    // ── preprocess: letterbox to input_w_ x input_h_ (aspect-preserving) ──
    const float scale = std::min(input_w_ / float(ow), input_h_ / float(oh));
    const int   nw = int(std::round(ow * scale));
    const int   nh = int(std::round(oh * scale));
    const int   px = (input_w_ - nw) / 2;
    const int   py = (input_h_ - nh) / 2;

    // Letterbox transform + proto geometry, so consumers can map original px
    // into the proto-resolution masks produced below (see SegResult).
    result.mask_width  = proto_w_;  result.mask_height  = proto_h_;
    result.input_width = input_w_;  result.input_height = input_h_;
    result.scale = scale;  result.pad_x = px;  result.pad_y = py;

    cv::resize(bgr, resized_, cv::Size(nw, nh));
    if (canvas_.empty())
        canvas_.create(input_h_, input_w_, CV_8UC3);
    canvas_.setTo(cv::Scalar(114, 114, 114));
    resized_.copyTo(canvas_(cv::Rect(px, py, nw, nh)));

    // BGR->RGB, /255, HWC->CHW into the reused blob (empty Size => no internal
    // resize; canvas_ is already input-sized), then into the pinned buffer.
    cv::dnn::blobFromImage(canvas_, blob_, 1.0 / 255.0, cv::Size(), cv::Scalar(),
                           /*swapRB=*/true, /*crop=*/false, CV_32F);
    std::memcpy(input_buf_.data(), blob_.ptr<float>(), input_buf_.size() * sizeof(float));
    pre_ms_.store(ms_since(t_pre), std::memory_order_relaxed);

    // ── inference ──
    const auto t_inf = clock::now();
    engine_.SetInputDataAsync(cfg_.input_name, input_buf_, stream_);
    if (!engine_.Enqueue(stream_)) {
        std::cerr << "[YoloSegEngine] Enqueue failed\n";
        return result;
    }
    engine_.GetOutputDataAsync(cfg_.det_name,   det_buf_,   stream_);
    engine_.GetOutputDataAsync(cfg_.proto_name, proto_buf_, stream_);
    cudaStreamSynchronize(stream_);
    inf_ms_.store(ms_since(t_inf), std::memory_order_relaxed);

    // ── postprocess (NMS-free) ──
    const auto t_post = clock::now();
    // Each detection row is [x1, y1, x2, y2, score, class, m0..m(C-1)] in the
    // letterboxed input's pixel scale. proto is [C, ph, pw]; the instance mask
    // is (coeffs . protos) thresholded at 0, kept at proto resolution (the
    // upsample to the frame is deferred to consumers — see SegResult).
    const int   num_masks   = proto_c_;
    const size_t proto_area = static_cast<size_t>(proto_h_) * proto_w_;
    // protos as a [C, ph*pw] matrix for the coeff x proto GEMM.
    const cv::Mat proto_mat(num_masks, int(proto_area), CV_32F, proto_buf_.data());
    const float sx = proto_w_ / float(input_w_), sy = proto_h_ / float(input_h_);

    // Pass 1: keep detections above threshold, gather their geometry + mask
    // coefficients into one [N, C] matrix. Boxes are in original px; pboxes are
    // the same boxes in proto coords (proto is aligned to the letterboxed input).
    struct Keep { cv::Rect box, pbox; float score; int class_id; };
    std::vector<Keep> keep;
    cv::Mat coeffs;   // [N, C], one row per kept detection

    for (int i = 0; i < det_count_; ++i) {
        const float* d = det_buf_.data() + size_t(i) * det_stride_;
        const float score = d[4];
        if (score < cfg_.score_threshold) continue;

        float x1 = (d[0] - px) / scale, y1 = (d[1] - py) / scale;
        float x2 = (d[2] - px) / scale, y2 = (d[3] - py) / scale;
        x1 = std::clamp(x1, 0.f, float(ow));  y1 = std::clamp(y1, 0.f, float(oh));
        x2 = std::clamp(x2, 0.f, float(ow));  y2 = std::clamp(y2, 0.f, float(oh));
        cv::Rect box(int(x1), int(y1), int(x2 - x1), int(y2 - y1));
        box &= cv::Rect(0, 0, ow, oh);
        if (box.width <= 0 || box.height <= 0) continue;

        cv::Rect pbox(int(std::floor(d[0] * sx)), int(std::floor(d[1] * sy)), 0, 0);
        pbox.width  = int(std::ceil(d[2] * sx)) - pbox.x;
        pbox.height = int(std::ceil(d[3] * sy)) - pbox.y;
        pbox &= cv::Rect(0, 0, proto_w_, proto_h_);
        if (pbox.width <= 0 || pbox.height <= 0) continue;

        coeffs.push_back(cv::Mat(1, num_masks, CV_32F, const_cast<float*>(d + 6)));
        keep.push_back({box, pbox, score, int(d[5])});
    }

    // Pass 2: one batched GEMM (coeffs [N,C] x protos [C,area] -> [N,area]),
    // then each mask is finished at proto resolution — crop to the box and
    // threshold the raw logit at 0 (sigmoid(x)>0.5 <=> x>0, so no exp needed).
    // No upsample, no full-frame alloc: this is Ultralytics process_mask with
    // upsample=False, so cost is O(proto area), independent of object size.
    if (!keep.empty()) {
        const cv::Mat masks_all = coeffs * proto_mat;   // [N, ph*pw] logits
        for (size_t k = 0; k < keep.size(); ++k) {
            const cv::Mat m(proto_h_, proto_w_, CV_32F,
                            const_cast<float*>(masks_all.ptr<float>(int(k))));

            // crop_mask: keep the box region (proto coords), zero elsewhere.
            cv::Mat mask = cv::Mat::zeros(proto_h_, proto_w_, CV_8U);
            cv::Mat bin = m(keep[k].pbox) > 0.0f;   // logit>0 -> CV_8U 0/255
            bin.copyTo(mask(keep[k].pbox));

            SegDetection det;
            det.box      = keep[k].box;
            det.score    = keep[k].score;
            det.class_id = keep[k].class_id;
            det.mask     = std::move(mask);
            result.detections.push_back(std::move(det));
        }
    }

    post_ms_.store(ms_since(t_post), std::memory_order_relaxed);
    return result;
}

} // namespace kist
