#include "segmentation/yolo_seg_engine.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <cuda_runtime_api.h>

#include <algorithm>
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

    // ── preprocess: letterbox to input_w_ x input_h_ (aspect-preserving) ──
    const float scale = std::min(input_w_ / float(ow), input_h_ / float(oh));
    const int   nw = int(std::round(ow * scale));
    const int   nh = int(std::round(oh * scale));
    const int   px = (input_w_ - nw) / 2;
    const int   py = (input_h_ - nh) / 2;

    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(nw, nh));
    cv::Mat canvas(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(px, py, nw, nh)));

    // BGR->RGB, /255, HWC->CHW into the pinned input buffer.
    cv::Mat blob;
    cv::dnn::blobFromImage(canvas, blob, 1.0 / 255.0,
                           cv::Size(input_w_, input_h_), cv::Scalar(),
                           /*swapRB=*/true, /*crop=*/false, CV_32F);
    std::memcpy(input_buf_.data(), blob.ptr<float>(), input_buf_.size() * sizeof(float));

    // ── inference ──
    engine_.SetInputDataAsync(cfg_.input_name, input_buf_, stream_);
    if (!engine_.Enqueue(stream_)) {
        std::cerr << "[YoloSegEngine] Enqueue failed\n";
        return result;
    }
    engine_.GetOutputDataAsync(cfg_.det_name,   det_buf_,   stream_);
    engine_.GetOutputDataAsync(cfg_.proto_name, proto_buf_, stream_);
    cudaStreamSynchronize(stream_);

    // ── postprocess (NMS-free) ──
    // Each detection row is [x1, y1, x2, y2, score, class, m0..m(C-1)] in the
    // letterboxed input's pixel scale. proto is [C, ph, pw]; the instance mask
    // is sigmoid(coeffs . protos), upsampled and un-letterboxed to the frame.
    const int   num_masks   = proto_c_;
    const size_t proto_area = static_cast<size_t>(proto_h_) * proto_w_;
    // protos as a [C, ph*pw] matrix for a fast coeff x proto GEMM.
    const cv::Mat proto_mat(num_masks, int(proto_area), CV_32F, proto_buf_.data());

    for (int i = 0; i < det_count_; ++i) {
        const float* d = det_buf_.data() + size_t(i) * det_stride_;
        const float score = d[4];
        if (score < cfg_.score_threshold) continue;

        // box: letterboxed input px -> original px
        float x1 = (d[0] - px) / scale, y1 = (d[1] - py) / scale;
        float x2 = (d[2] - px) / scale, y2 = (d[3] - py) / scale;
        x1 = std::clamp(x1, 0.f, float(ow));  y1 = std::clamp(y1, 0.f, float(oh));
        x2 = std::clamp(x2, 0.f, float(ow));  y2 = std::clamp(y2, 0.f, float(oh));
        cv::Rect box(int(x1), int(y1), int(x2 - x1), int(y2 - y1));
        box &= cv::Rect(0, 0, ow, oh);
        if (box.width <= 0 || box.height <= 0) continue;

        SegDetection det;
        det.box      = box;
        det.score    = score;
        det.class_id = int(d[5]);

        // mask: coeffs (1 x C) * protos (C x area) -> (1 x area) -> [ph, pw]
        const cv::Mat coeff(1, num_masks, CV_32F, const_cast<float*>(d + 6));
        cv::Mat m = coeff * proto_mat;                 // [1, ph*pw]
        m = m.reshape(1, proto_h_);                    // [ph, pw]
        // sigmoid
        cv::exp(-m, m);
        m = 1.0 / (1.0 + m);

        // upsample to input, strip letterbox padding, resize to original
        cv::Mat m_in;   cv::resize(m, m_in, cv::Size(input_w_, input_h_), 0, 0, cv::INTER_LINEAR);
        cv::Mat m_crop = m_in(cv::Rect(px, py, nw, nh));
        cv::Mat m_orig; cv::resize(m_crop, m_orig, cv::Size(ow, oh), 0, 0, cv::INTER_LINEAR);
        cv::Mat bin;    cv::threshold(m_orig, bin, cfg_.mask_threshold, 255, cv::THRESH_BINARY);
        bin.convertTo(bin, CV_8U);

        // keep only the part inside the box
        det.mask = cv::Mat::zeros(oh, ow, CV_8U);
        bin(box).copyTo(det.mask(box));

        result.detections.push_back(std::move(det));
    }

    return result;
}

} // namespace kist
