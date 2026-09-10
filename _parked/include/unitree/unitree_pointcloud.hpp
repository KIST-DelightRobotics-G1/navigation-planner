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

    // Optional per-point channels (Mid-360 carries all three). Each is either
    // empty (field absent) or exactly point_count() long, kept in LOCKSTEP with
    // xyz — a dropped non-finite point is absent from every channel.
    std::vector<float>    intensity;   // Livox reflectivity
    std::vector<uint16_t> ring;        // scan-line index
    std::vector<float>    time;        // per-point time offset in NANOSECONDS
                                       // relative to stamp_ns (deskew / LIO channel);
                                       // absolute t_i = stamp_ns + int64(time[i]).
                                       // Measured span ~93 ms = one 10 Hz frame.

    std::size_t point_count() const noexcept { return xyz.size() / 3; }
    bool has_intensity() const noexcept { return !intensity.empty(); }
    bool has_ring()      const noexcept { return !ring.empty(); }
    bool has_time()      const noexcept { return !time.empty(); }
};

} // namespace kist
