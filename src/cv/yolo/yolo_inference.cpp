#include "cv/yolo/yolo_inference.hpp"

#include <cuda_runtime_api.h>

#include <iostream>

namespace kist {

YoloInference::~YoloInference() {
    if (stream_) {
        cudaStreamDestroy(stream_);
        stream_ = nullptr;
    }
}

bool YoloInference::init(const std::string& onnx_path) {
    // Build (or load cached) a FP16 engine from the ONNX.
    Options opts;
    opts.precision = Precision::FP16;
    opts.deviceID  = 0;
    std::string trt_path;
    if (!ConvertONNXToTRT(opts, onnx_path, trt_path)) {
        std::cerr << "[YoloInference] ConvertONNXToTRT failed: " << onnx_path << "\n";
        return false;
    }
    if (!engine_.Initialize(trt_path, 0) || !engine_.InitInputs()) {
        std::cerr << "[YoloInference] engine init failed: " << trt_path << "\n";
        return false;
    }
    if (cudaStreamCreate(&stream_) != cudaSuccess) {
        std::cerr << "[YoloInference] cudaStreamCreate failed\n";
        return false;
    }
    initialized_ = true;
    return true;
}

std::vector<int> YoloInference::tensor_shape(const std::string& name) const {
    std::vector<int64_t> shape;
    std::vector<int> out;
    if (engine_.GetTensorShape(name, shape))
        for (auto d : shape) out.push_back(static_cast<int>(d));
    return out;
}

void YoloInference::set_input_async(const std::string& name, const TPinnedVector<float>& buf) {
    engine_.SetInputDataAsync(name, buf, stream_);
}

bool YoloInference::enqueue() {
    return engine_.Enqueue(stream_);
}

void YoloInference::get_output_async(const std::string& name, TPinnedVector<float>& buf) {
    engine_.GetOutputDataAsync(name, buf, stream_);
}

void YoloInference::get_output_async(const std::string& name, void* dst, size_t byteCount) {
    engine_.GetOutputDataAsync(name, dst, byteCount, stream_);
}

void YoloInference::sync() {
    cudaStreamSynchronize(stream_);
}

void* YoloInference::output_device_ptr(const std::string& name) {
    return engine_.GetOutputDevicePtr(name);
}

} // namespace kist
