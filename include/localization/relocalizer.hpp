#pragma once

// Relocalizer — estimates the map->odom transform by registering a bounded local submap
// (accumulated from the LIO registered scans, odom frame) against a prior PCD map (map frame).
// This fills the `map -> odom` edge of the frame tree (see include/frames/frame_ids.hpp), which
// reconciles the drifting/boot-relative LIO odom with the fixed prior map so that fixed
// destinations (config/destinations.yaml, map frame) land on the same physical spot every run.
//
// v0 (this): seed a rough T_map_odom, then GICP refines + tracks it. Automatic global init
// (place recognition -> coarse reg) and the GLOBAL_SEARCH<->TRACKING failure state machine are
// v1. PCL is hidden behind a PIMPL so consumers don't pull PCL headers.
//
//   source = current submap (odom)  --GICP-->  target = prior map (map)
//   getFinalTransformation() == T_map_odom  (init guess = the current estimate)

#include "lio/lio_cloud.hpp"

#include <Eigen/Geometry>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace kist {

struct RelocConfig {
    float  map_voxel_m     = 0.10f;  // prior-map downsample leaf (m)
    float  submap_voxel_m  = 0.10f;  // submap downsample leaf (m)
    int    submap_scans    = 40;     // rolling window: keep the most recent N registered scans
    double gicp_max_corr_m = 1.0;    // GICP max correspondence distance (m)
    int    gicp_max_iter   = 50;     // GICP max iterations
    double fitness_max      = 0.30;  // accept only if GICP fitness (mean sq err, m^2) <= this
    double max_jump_m       = 1.0;   // reject a correction translating T_map_odom more than this
    double max_yaw_deg      = 30.0;  // reject a solution whose heading deviates from the SEED yaw
                                     // by more than this (kills 180deg flips in symmetric scenes;
                                     // relies on a roughly-correct seed yaw, i.e. a fixed boot
                                     // heading). 0 = disabled.
};

class Relocalizer {
public:
    Relocalizer();
    ~Relocalizer();
    Relocalizer(const Relocalizer&) = delete;
    Relocalizer& operator=(const Relocalizer&) = delete;

    bool load_prior(const std::string& pcd_path);   // load + downsample the prior map (map frame)
    void set_config(const RelocConfig& c);
    const RelocConfig& config() const { return cfg_; }

    void        add_scan(const LioCloud& scan);      // accumulate into the rolling submap (odom)
    std::size_t submap_size() const;                 // downsampled submap point count
    std::size_t prior_size()  const;                 // downsampled prior-map point count

    // Downsampled prior map as flat xyz (map frame) — for the rviz overlay (drawn in odom by
    // transforming with T_odom_map, so it sits on the live obstacle grid when localised).
    std::vector<float> prior_xyz() const;

    void seed(const Eigen::Matrix4f& T_map_odom);    // initial guess (map<-odom)
    bool has_estimate() const { return seeded_; }
    Eigen::Matrix4f T_map_odom() const { return T_map_odom_; }

    // One GICP step: align the current submap (source, odom) to the prior map (target, map),
    // starting from the current T_map_odom. On accept (converged + fitness + jump gates) the
    // estimate is updated. Returns true if accepted; `fitness_out` (opt) gets the GICP score
    // (>=0 when GICP ran, <0 if it did not converge).
    bool align(double* fitness_out = nullptr);

    // Debug: write prior map + current submap transformed by T_map_odom into one PCD so the
    // overlap can be eyeballed in pcl_viewer (submap tagged high-intensity to colour-split).
    bool dump_overlay(const std::string& pcd_path) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    RelocConfig     cfg_;
    Eigen::Matrix4f T_map_odom_ = Eigen::Matrix4f::Identity();
    bool            seeded_ = false;
    float           seed_yaw_ = 0.f;   // heading of the seed — the yaw gate's reference
};

}  // namespace kist
