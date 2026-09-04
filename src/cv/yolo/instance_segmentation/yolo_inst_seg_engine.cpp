#include "cv/yolo/instance_segmentation/yolo_inst_seg_engine.hpp"

#include <chrono>
#include <iostream>

namespace kist {

bool YoloInstSegEngine::init(const Config& cfg) {
    cfg_ = cfg;

    if (!infer_.init(cfg_.onnx_path)) {
        std::cerr << "[YoloInstSegEngine] inference init failed: " << cfg_.onnx_path << "\n";
        return false;
    }

    // Resolve shapes from the engine (don't hardcode).
    auto in_s    = infer_.tensor_shape(cfg_.input_name);   // [1, 3, H, W]
    auto det_s   = infer_.tensor_shape(cfg_.det_name);     // [1, N, S]
    auto proto_s = infer_.tensor_shape(cfg_.proto_name);   // [1, C, ph, pw]
    if (in_s.size() != 4 || det_s.size() != 3 || proto_s.size() != 4) {
        std::cerr << "[YoloInstSegEngine] unexpected tensor ranks (in=" << in_s.size()
                  << " det=" << det_s.size() << " proto=" << proto_s.size() << ")\n";
        return false;
    }
    input_h_    = in_s[2];    input_w_    = in_s[3];
    det_count_  = det_s[1];   det_stride_ = det_s[2];
    proto_c_    = proto_s[1]; proto_h_    = proto_s[2]; proto_w_ = proto_s[3];

    input_buf_.resize(static_cast<size_t>(3) * input_h_ * input_w_);
    det_buf_.resize(static_cast<size_t>(det_count_) * det_stride_);

    // GPU postprocess (cuBLAS GEMM on the proto tensor, kept on-device) — shares
    // the inference stream so it runs right after the outputs are ready.
    if (!post_.init(det_count_, det_stride_, proto_c_, proto_h_, proto_w_,
                    infer_.stream())) {
        std::cerr << "[YoloInstSegEngine] postprocess init failed\n";
        return false;
    }

    std::cout << "[YoloInstSegEngine] ready: input " << input_w_ << "x" << input_h_
              << ", det [" << det_count_ << "," << det_stride_ << "]"
              << ", proto [" << proto_c_ << "," << proto_h_ << "," << proto_w_ << "]\n";
    initialized_ = true;
    return true;
}

InstSegFrame YoloInstSegEngine::infer(const cv::Mat& bgr, int64_t stamp_ns) {
    InstSegFrame result;
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

    // ── inference (generic) ── only the detection tensor comes to the host; the
    // prototype tensor stays on the GPU for the cuBLAS postprocess below.
    const auto t_inf = clock::now();
    infer_.set_input_async(cfg_.input_name, input_buf_);
    if (!infer_.enqueue()) {
        std::cerr << "[YoloInstSegEngine] enqueue failed\n";
        return result;
    }
    infer_.get_output_async(cfg_.det_name, det_buf_);
    infer_.sync();
    inf_ms_.store(ms_since(t_inf), std::memory_order_relaxed);

    // ── postprocess (cuBLAS GEMM) ──
    const auto t_post = clock::now();
    post_.run(det_buf_.data(), infer_.output_device_ptr(cfg_.proto_name),
              lb, bgr.cols, bgr.rows, cfg_.score_threshold, cfg_.mask_erode_px, result);
    post_ms_.store(ms_since(t_post), std::memory_order_relaxed);

    return result;
}

} // namespace kist
