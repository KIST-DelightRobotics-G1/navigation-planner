#pragma once

// A Transform plus its coordinate-system metadata: the timestamp it is valid at
// and the (parent, child) frames it connects. The math stays in Transform; this
// is what the transform tree stores/passes so a transform always knows which
// edge (parent <- child) and when it is. T_parent_child follows the T_A_B
// convention: p_parent = T_parent_child * p_child.

#include "frames/frame_ids.hpp"
#include "transforms/transform.hpp"

#include <cstdint>

namespace kist {

struct StampedTransform {
    int64_t   stamp_ns = 0;
    FrameId   parent   = FrameId::Map;
    FrameId   child    = FrameId::Map;
    Transform T_parent_child;
};

} // namespace kist
