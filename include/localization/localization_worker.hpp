#pragma once

// LocalizationWorker — the localization thread: accumulate registered LIO scans into the
// relocalizer's rolling submap and, at a low rate, GICP-align it to the prior map, publishing
// the resulting map->odom (MapOdom) to a buffer. Owns its thread + loop; the registration LOGIC
// stays in Relocalizer. v0: seeded once at start (config); auto global init / re-init = later.

#include "common/data_buffer.hpp"
#include "lio/lio_receiver.hpp"
#include "localization/map_odom.hpp"
#include "localization/relocalizer.hpp"

#include <Eigen/Geometry>
#include <atomic>
#include <string>
#include <thread>

namespace kist {

class LocalizationWorker {
public:
    ~LocalizationWorker() { stop(); }

    // Load the prior map + seed the estimate, then start tracking. `out` receives a MapOdom on
    // each accepted align. Returns false if the prior map cannot be loaded (worker not started).
    bool start(LioReceiver& rx, const std::string& prior_pcd, const Eigen::Matrix4f& seed,
               DataBuffer<MapOdom>& out, const RelocConfig& rcfg = {});
    void stop();

    std::vector<float> prior_xyz() const { return reloc_.prior_xyz(); }   // map frame, for rviz overlay

private:
    void run();

    Relocalizer          reloc_;
    LioReceiver*         rx_  = nullptr;
    DataBuffer<MapOdom>* out_ = nullptr;

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
