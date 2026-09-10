#pragma once

// LioCloud — one registered LiDAR scan already in the odom frame (from /cloud_registered_1).
// This is the engine's motion-compensated output — the stable view occupancy consumes.
// Flattened xyz; intensity optional.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace kist {

struct LioCloud {
    int64_t             stamp_ns = 0;
    std::string         frame_id;          // "camera_init" == odom
    std::vector<float>  xyz;               // x0 y0 z0 x1 y1 z1 ...
    std::vector<float>  intensity;         // empty if the field is absent

    std::size_t point_count()  const noexcept { return xyz.size() / 3; }
    bool        has_intensity() const noexcept { return !intensity.empty(); }
};

} // namespace kist
