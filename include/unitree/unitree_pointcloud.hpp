#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace kist {

// One LiDAR frame — the output of UnitreePointCloudReader. Lives next
// to its reader, same pairing as gearsonic's unitree_state.hpp /
// unitree_state_reader.hpp.
struct UnitreePointCloud {
    // Sensor timestamp from the PointCloud2 header.
    int64_t stamp_ns = 0;

    // Coordinate frame; "livox_frame" as published by the G1's internal
    // driver (sensor frame, no motion compensation — the onboard
    // lidar_node relay rewrites this to "utlidar_lidar", we don't).
    std::string frame_id;

    // float32[N, 3], flattened: x0 y0 z0 x1 y1 z1 ...
    std::vector<float> xyz;

    std::size_t point_count() const noexcept { return xyz.size() / 3; }
};

} // namespace kist
