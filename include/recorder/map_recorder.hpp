#pragma once

// MapRecorder — the map-recording worker. Consumes the LIO registered scans (rt/cloud_registered_1,
// odom frame) into a bounded voxel grid, and captures the map-origin UWB + the odom pose P_B at that
// instant (so the deploy seed can double-transform UWB->map correctly). Owns its thread + the
// accumulation loop; the tool just wires the readers and manages lifecycle (start / stop / save) —
// the same worker split as the runtime stages (see e.g. LocalizationWorker).

#include "lio/lio_receiver.hpp"
#include "localization/uwb_receiver.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <unordered_map>

namespace kist {

struct MapRecorderConfig {
    float voxel_m      = 0.05f;   // voxel leaf for map accumulation (m)
    float origin_avg_s = 5.0f;    // average the map-origin UWB + P_B over this window (s) to cut
                                  // the UWB jitter (~50 samples at 10 Hz). Hold the robot still for
                                  // the first ~origin_avg_s so the average is a clean single spot.
};

class MapRecorder {
public:
    MapRecorder() = default;
    ~MapRecorder() { stop(); }
    MapRecorder(const MapRecorder&) = delete;
    MapRecorder& operator=(const MapRecorder&) = delete;

    // Start the accumulation thread. `uwb` may be null (no UWB -> no sidecar written on save).
    bool start(LioReceiver& rx, UwbReceiver* uwb, const MapRecorderConfig& cfg = {});
    void stop();

    // Progress snapshots (lock-free) for the tool's status line.
    uint64_t    scans()      const { return scans_.load(); }
    uint64_t    raw_points() const { return raw_points_.load(); }
    std::size_t voxels()     const { return voxel_count_.load(); }
    bool        has_origin() const { return origin_have_.load(); }

    // Write the accumulated map to `pcd_path` (ASCII PCD: x y z intensity) + the UWB sidecar
    // (`pcd_path` with .pcd -> .uwb; two lines: UWB, then P_B). Call after stop(). Returns false if
    // nothing was accumulated or the file cannot be written.
    bool save(const std::string& pcd_path) const;

private:
    void run();

    struct Vox { double sx = 0, sy = 0, sz = 0, si = 0; uint32_t n = 0; };

    LioReceiver*      rx_  = nullptr;
    UwbReceiver*      uwb_ = nullptr;
    MapRecorderConfig cfg_;

    std::unordered_map<int64_t, Vox> grid_;   // owned by the run thread; read by save() after join
    std::atomic<std::size_t> voxel_count_{0};
    std::atomic<uint64_t>    scans_{0}, raw_points_{0};

    // Map-origin capture (UWB + odom pose P_B at that instant).
    std::atomic<bool> origin_have_{false};
    float uwb_ox_ = 0.f, uwb_oy_ = 0.f;
    float pb_x_ = 0.f, pb_y_ = 0.f, pb_yaw_ = 0.f;

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
