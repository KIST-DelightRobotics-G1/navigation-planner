#pragma once

// LocalizationWorker — the localization thread. Two phases, both in one 1 Hz loop:
//   GLOBAL-INIT : accumulate registered scans -> yaw-search global_init (UWB xy + robust sweep) ->
//                 seed the EKF once a confident/stable lock is found.
//   TRACKING    : 3-state EKF (LocalizationFilter) fuses the GICP measurement (map<-odom geometry)
//                 with the UWB position each cycle -> publishes T_map_odom (MapOdom). On an extreme
//                 UWB disagreement (LOST) it drops the lock and returns to GLOBAL-INIT.
// Owns its thread; the registration/EKF LOGIC lives in Relocalizer + LocalizationFilter (pure).

#include "common/data_buffer.hpp"
#include "lio/lio_receiver.hpp"
#include "lio/lio_transform_producer.hpp"
#include "localization/localization_filter.hpp"
#include "localization/map_odom.hpp"
#include "localization/relocalizer.hpp"
#include "localization/uwb_receiver.hpp"

#include <Eigen/Geometry>
#include <atomic>
#include <string>
#include <thread>

namespace kist {

class LocalizationWorker {
public:
    ~LocalizationWorker() { stop(); }

    // rx        : registered cloud (submap) + LIO odom.
    // prod      : robot transforms (T_odom_pelvis) — for the UWB measurement equation.
    // uwb       : UWB fixes (nullptr -> no fusion; falls back to the fixed seed).
    // prior_pcd : prior map; sidecar : maps/map.uwb (UWB + P_B) for the deploy seed.
    // map_uwb_yaw_rad, tag_in_pelvis : UWB->map calibration + antenna offset in base.
    // fallback_seed : fixed T_map_odom used when UWB is unavailable.
    // Returns false if the prior map cannot be loaded (worker not started).
    bool start(LioReceiver& rx, LioTransformProducer& prod, UwbReceiver* uwb,
               const std::string& prior_pcd, const std::string& sidecar,
               float map_uwb_yaw_rad, const Eigen::Vector3f& tag_in_pelvis,
               const Eigen::Matrix4f& fallback_seed, DataBuffer<MapOdom>& out,
               const RelocConfig& rcfg = {}, const LocFilterConfig& fcfg = {});
    void stop();

    std::vector<float> prior_xyz() const { return reloc_.prior_xyz(); }   // map frame, for rviz overlay

private:
    void run();
    // Resolve the live UWB antenna xy in map (have_uwb set if a fix was available) + the odom<-base
    // pose. Returns have_odom (the odom pose is essential for both phases).
    bool resolve_inputs(Eigen::Vector2f& uwb_xy, bool& have_uwb, Eigen::Matrix4f& T_odom_base) const;

    Relocalizer        reloc_;
    LocalizationFilter ekf_;

    LioReceiver*          rx_   = nullptr;
    LioTransformProducer* prod_ = nullptr;
    UwbReceiver*          uwb_  = nullptr;
    DataBuffer<MapOdom>*  out_  = nullptr;

    std::string     sidecar_;
    float           map_uwb_yaw_ = 0.f;
    Eigen::Vector3f tag_in_pelvis_ = Eigen::Vector3f::Zero();
    Eigen::Matrix4f fallback_seed_ = Eigen::Matrix4f::Identity();

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
