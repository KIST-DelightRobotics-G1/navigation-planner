#pragma once

namespace kist {

// Per-stage wall-clock cost of the last infer() call (ms). Diagnostic only —
// lets a runner see whether a frame is preprocess-, GPU-, or postprocess-bound.
// `infer` covers the whole H2D + enqueue + D2H block up to the stream sync.
// Shared by every YOLO task engine (instance / semantic).
struct YoloStageTimings {
    double preprocess_ms  = 0.0;
    double infer_ms       = 0.0;
    double postprocess_ms = 0.0;
};

} // namespace kist
