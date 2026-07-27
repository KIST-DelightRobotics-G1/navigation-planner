#pragma once

#include <cstdint>

namespace kist {

// PoseFilter's UWB input contract (PoseFilter::uwb_buf).
//
// Producer contract (the wiring point pushes these — the standalone runner
// and the future orchestrator both bridge kist-ext-sensor-io's UwbReceiver
// into this buffer via its set_on_position hook):
//   - valid fixes only (DWM quality > 0); no placeholder pushes
//   - UWB local frame (anchor coordinates); trilateration only, no heading
//   - goes silent on lost fix / dead serial -> the buffer stales (empty-
//     buffer principle), which naturally stops UWB updates
//
// navigation-planner's own type, deliberately NOT ext-sensor-io's UwbPosition:
// keeping the filter core on its own struct lets kalman_filter build without
// the ext-sensor-io checkout, so the ext dependency lives only at the wiring
// callback (which converts UwbPosition -> UwbFix). The EKF is 2D, so z is
// dropped here; only x/y reach the filter.
struct UwbFix {
    int64_t stamp_ns = 0;   // publisher timestamp (dedup key)
    float   x = 0.0f;       // UWB local frame (m)
    float   y = 0.0f;
};

} // namespace kist
