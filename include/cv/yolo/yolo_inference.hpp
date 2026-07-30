#pragma once

#include "tensorrt/InferenceEngine.h"   // TRTInferenceEngine, TPinnedVector

#include <string>
#include <vector>

// Forward-declare the CUDA stream type (avoid pulling cuda headers here).
typedef struct CUstream_st* cudaStream_t;

namespace kist {

// Generic YOLO TensorRT inference wrapper: builds/loads a FP16 engine from an
// ONNX file (via the vendored TRTInferenceEngine) and runs one forward pass on
// its own CUDA stream. Model-agnostic — the caller owns the pinned I/O buffers
// and addresses tensors by name, so it serves detection / seg / pose alike.
class YoloInference {
public:
    YoloInference() = default;
    ~YoloInference();

    YoloInference(const YoloInference&) = delete;
    YoloInference& operator=(const YoloInference&) = delete;

    // Build (or load cached) the engine and create the stream.
    bool init(const std::string& onnx_path);

    // Per-dim sizes of a tensor from the engine; empty on unknown name.
    std::vector<int> tensor_shape(const std::string& name) const;

    // One forward pass, timed by the caller: upload the (already-filled) input
    // (H2D, async), enqueue, then download each requested output (D2H, async).
    // Call sync() before reading the outputs.
    void set_input_async(const std::string& name, const TPinnedVector<float>& buf);
    bool enqueue();
    void get_output_async(const std::string& name, TPinnedVector<float>& buf);
    void sync();

    // Raw device pointer of an output tensor (valid after enqueue+sync, until the
    // next enqueue) — lets a caller run cuBLAS/CUDA postprocessing on the output
    // without a device->host copy.
    void* output_device_ptr(const std::string& name);

    cudaStream_t stream() const { return stream_; }
    bool initialized() const { return initialized_; }

private:
    TRTInferenceEngine engine_;
    cudaStream_t       stream_ = nullptr;
    bool               initialized_ = false;
};

} // namespace kist
