#pragma once

// MapRecorderSystem — the map-recording deployment facade / ASSEMBLER (mirrors NavSystem). It owns
// the LIO + UWB readers and the MapRecorder worker, wires them together, and start/stops them.
// There are NO loops here — the MapRecorder worker owns its thread + accumulation loop. The entry
// binary (kist-map-recorder) just runs the lifecycle and, on quit, calls save().

#include "lio/lio_receiver.hpp"
#include "localization/uwb_receiver.hpp"
#include "recorder/map_recorder.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

namespace kist {

class MapRecorderSystem {
public:
    MapRecorderSystem() = default;
    ~MapRecorderSystem() { stop(); }
    MapRecorderSystem(const MapRecorderSystem&) = delete;
    MapRecorderSystem& operator=(const MapRecorderSystem&) = delete;

    bool start(const std::string& config_path);   // load config + DDS, start readers + the worker
    void stop();

    void install_signal_handlers();
    void request_quit() { quit_ = true; }
    bool quit_requested() const { return quit_; }

    // Progress snapshots (for the entry's status line) — forwarded from the worker.
    uint64_t    scans()      const { return rec_.scans(); }
    uint64_t    raw_points() const { return rec_.raw_points(); }
    std::size_t voxels()     const { return rec_.voxels(); }
    bool        has_origin() const { return rec_.has_origin(); }

    // Write the accumulated map (+ .uwb sidecar) to pcd_path. Call after stop().
    bool save(const std::string& pcd_path) const { return rec_.save(pcd_path); }

private:
    LioReceiver rx_;
    UwbReceiver uwb_;
    MapRecorder rec_;

    std::atomic<bool> quit_{false};
    bool rx_started_{false}, uwb_started_{false};
};

} // namespace kist
