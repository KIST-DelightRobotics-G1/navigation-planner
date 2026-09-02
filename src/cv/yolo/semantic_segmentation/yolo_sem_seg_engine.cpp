#include "cv/yolo/semantic_segmentation/yolo_sem_seg_engine.hpp"

#include "cv/yolo/semantic_segmentation/yolo_sem_seg_postprocess.hpp"

#include <chrono>
#include <iostream>

namespace kist {

bool YoloSemSegEngine::init(const Config& cfg) {
    cfg_ = cfg;

    if (!infer_.init(cfg_.onnx_path)) {
        std::cerr << "[YoloSemSegEngine] inference init failed: " << cfg_.onnx_path << "\n";
        return false;
    }

    auto in_s  = infer_.tensor_shape(cfg_.input_name);    // [1, 3, H, W]
    auto out_s = infer_.tensor_shape(cfg_.output_name);   // [1, H, W]
    if (in_s.size() != 4 || out_s.size() != 3) {
        std::cerr << "[YoloSemSegEngine] unexpected tensor ranks (in=" << in_s.size()
                  << " out=" << out_s.size() << ")\n";
        return false;
    }
    input_h_ = in_s[2];  input_w_ = in_s[3];
    out_h_   = out_s[1]; out_w_   = out_s[2];

    input_buf_.resize(static_cast<size_t>(3) * input_h_ * input_w_);
    class_buf_.resize(static_cast<size_t>(out_h_) * out_w_);

    std::cout << "[YoloSemSegEngine] ready: input " << input_w_ << "x" << input_h_
              << ", class map " << out_w_ << "x" << out_h_ << "\n";
    initialized_ = true;
    return true;
}

SemSegFrame YoloSemSegEngine::infer(const cv::Mat& bgr, int64_t stamp_ns) {
    SemSegFrame result;
    if (!initialized_ || bgr.empty()) return result;
    result.stamp_ns = stamp_ns;

    using clock = std::chrono::steady_clock;
    auto ms_since = [](clock::time_point t) {
        return std::chrono::duration<double, std::milli>(clock::now() - t).count();
    };

    // ── preprocess (generic) ──
    const auto t_pre = clock::now();
    LetterboxTransform lb;
    yolo_preprocess(bgr, input_w_, input_h_, input_buf_.data(), lb, pre_scratch_);
    pre_ms_.store(ms_since(t_pre), std::memory_order_relaxed);

    // ── inference (generic) ── the output is a UINT8 class-id map, so D2H it
    // raw into the byte scratch.
    const auto t_inf = clock::now();
    infer_.set_input_async(cfg_.input_name, input_buf_);
    if (!infer_.enqueue()) {
        std::cerr << "[YoloSemSegEngine] enqueue failed\n";
        return result;
    }
    infer_.get_output_async(cfg_.output_name, class_buf_.data(), class_buf_.size());
    infer_.sync();
    inf_ms_.store(ms_since(t_inf), std::memory_order_relaxed);

    // ── postprocess (wrap the class map + letterbox transform) ──
    const auto t_post = clock::now();
    yolo_sem_seg_postprocess(class_buf_.data(), out_w_, out_h_, lb, bgr.cols, bgr.rows, result);
    post_ms_.store(ms_since(t_post), std::memory_order_relaxed);

    return result;
}

} // namespace kist
