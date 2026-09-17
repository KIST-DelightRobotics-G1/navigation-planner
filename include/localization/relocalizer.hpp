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
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace kist {

struct RelocConfig {
    float  map_voxel_m     = 0.05f;  // prior-map downsample leaf (m) — matches the recording/occupancy
                                     // resolution for a tighter fit (lower median, sharper yaw). Coarser
                                     // = faster GICP but mushier. Fixed here (edit + rebuild to change).
    float  submap_voxel_m  = 0.05f;  // submap downsample leaf (m)
    int    submap_scans    = 40;     // rolling window: keep the most recent N registered scans
    double gicp_max_corr_m = 1.0;    // GICP max correspondence distance (m)
    int    gicp_max_iter   = 50;     // GICP max iterations
    double fitness_max      = 0.30;  // accept only if GICP fitness (mean sq err, m^2) <= this
    double max_jump_m       = 1.0;   // reject a correction translating T_map_odom more than this
    double max_yaw_deg      = 30.0;  // reject a solution whose heading deviates from the SEED yaw
                                     // by more than this (kills 180deg flips in symmetric scenes;
                                     // relies on a roughly-correct seed yaw, i.e. a fixed boot
                                     // heading). 0 = disabled.

    // --- Global yaw-search init (UWB pins the base xy; yaw is the only real unknown). A robust
    // correlative scan-match over yaw replaces a trusted seed heading, so the robot may boot at an
    // arbitrary yaw. See Relocalizer::global_init. ---
    float  gi_inlier_thresh_m  = 0.30f;  // NN dist (submap->prior) below which a point is an inlier
    float  gi_yaw_step_deg     = 10.0f;  // coarse sweep step (0..360)
    int    gi_coarse_stride    = 4;      // subsample submap 1/stride for the cheap coarse score
    float  gi_peak_sep_deg     = 40.0f;  // peaks closer than this are the same hypothesis (for margin)
    float  gi_min_inlier_ratio = 0.50f;  // reject init if the best peak's inlier ratio is below this
    float  gi_min_confidence   = 0.25f;  // reject init if (best-second)/best margin is below this
    int    gi_top_k            = 3;      // GICP-refine this many distinct peaks (Phase 2)
    float  gi_xy_gate_m        = 0.30f;  // refine xy drift from UWB beyond this -> clamp (Phase 2)
    // Temporal-consistency accept: a correct yaw at a weakly-constrained spot (few walls) may have low
    // single-frame confidence but stays stable frame-to-frame. Accept a low-confidence result if the
    // best yaw has been within +/- tol across this many consecutive frames (still needs the inlier gate).
    int    gi_stable_frames    = 5;      // frames the best yaw must be stable over to lock on stability
    float  gi_stable_yaw_tol_deg = 3.0f; // max spread of the recent best yaws to count as "stable"
    // Floor/ceiling removal for the yaw score: the floor matches the prior at ANY yaw (and a dense
    // prior + a generous inlier threshold makes it worse), washing out the yaw signal — only walls
    // / vertical structure constrain yaw. We keep a floor-filtered copy of the prior + submap FOR
    // SCORING ONLY (GICP tracking + overlay still use the full clouds). Floor z is found per-cloud
    // by a z-histogram, so it works in either frame (prior=map, submap=odom).
    bool   gi_floor_filter    = true;   // drop floor+ceiling before scoring
    float  gi_floor_band_m    = 0.30f;  // remove points within this height above the detected floor
    float  gi_wall_top_m      = 2.50f;  // keep up to this height above the floor (drops the ceiling)
};

// Result of a global (yaw-search) initialization. ok == accepted (enough inlier support + a clear
// margin over the next hypothesis). On accept the Relocalizer has adopted T_map_odom as its estimate.
struct GlobalInitResult {
    Eigen::Matrix4f T_map_odom  = Eigen::Matrix4f::Identity();
    float           inlier_ratio = 0.f;   // best hypothesis' inlier ratio (0..1)
    float           confidence   = 0.f;   // (best - second_peak) / best, over well-separated peaks
    float           yaw_deg      = 0.f;   // winning heading (map<-odom base), deg
    bool            ok           = false;
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

    // The current submap transformed by an arbitrary map<-odom T, as flat xyz (map frame) — for
    // visualizing a global-init hypothesis: overlay this against prior_xyz() (both map frame); if
    // the yaw is right the submap walls sit on the prior walls.
    std::vector<float> submap_xyz(const Eigen::Matrix4f& T_map_odom) const;

    void seed(const Eigen::Matrix4f& T_map_odom);    // initial guess (map<-odom)
    bool has_estimate() const { return seeded_; }
    Eigen::Matrix4f T_map_odom() const { return T_map_odom_; }

    // One GICP step: align the current submap (source, odom) to the prior map (target, map),
    // starting from the current T_map_odom. On accept (converged + fitness + jump gates) the
    // estimate is updated. Returns true if accepted; `fitness_out` (opt) gets the GICP score
    // (>=0 when GICP ran, <0 if it did not converge).
    bool align(double* fitness_out = nullptr);

    // Global initialization by yaw search. UWB pins the base xy in map; the base heading (yaw) is
    // unknown, so we sweep yaw over 0..360, score each candidate by robust point-to-prior overlap
    // (inlier ratio; people/clutter fall outside the inlier threshold and drop out), coarsen->refine
    // the winning yaw, and require a clear margin over the next well-separated hypothesis (180deg
    // symmetry guard). On accept, adopts T_map_odom + its heading as the estimate/seed.
    //   tag_xy_map    : the UWB tag's xy in the map frame (R(map_uwb_yaw)*(uwb_now - map_origin))
    //   tag_in_pelvis : UWB tag offset in the pelvis frame (x fwd, y left, z up), applied per yaw
    //   T_odom_base   : the LIO base pose (odom<-pelvis) captured at the same instant as the UWB fix
    // Phase 1: score-only (no GICP refine yet); Phase 2 adds top-K bounded-GICP refinement.
    GlobalInitResult global_init(const Eigen::Vector2f& tag_xy_map,
                                 const Eigen::Vector3f& tag_in_pelvis,
                                 const Eigen::Matrix4f& T_odom_base,
                                 bool verbose = false);   // verbose = print the full coarse yaw curve

    // Score ONE manual yaw hypothesis (same framing as global_init: UWB pins base xy, tag offset
    // rotated per yaw, odom chained off the base). For interactive alignment — sweep the yaw by hand
    // and read the overlap. Returns the inlier ratio; T_out (opt) gets the candidate map<-odom.
    float score_yaw(const Eigen::Vector2f& tag_xy_map, const Eigen::Vector3f& tag_in_pelvis,
                    const Eigen::Matrix4f& T_odom_base, float yaw_deg,
                    Eigen::Matrix4f* T_out = nullptr, float* median_out = nullptr);

    // Debug: write prior map + current submap transformed by T_map_odom into one PCD so the
    // overlap can be eyeballed in pcl_viewer (submap tagged high-intensity to colour-split).
    bool dump_overlay(const std::string& pcd_path) const;                 // uses the current estimate
    bool dump_overlay(const std::string& pcd_path, const Eigen::Matrix4f& T_map_odom) const;

private:
    void ensure_submap();                                // rebuild merged+downsampled submap if dirty
    // Robust overlap score of a candidate map<-odom transform: fraction of submap points whose
    // nearest prior point is within gi_inlier_thresh_m (stride subsamples for speed). median_out
    // (opt) gets the median inlier distance (tie-break / diagnostics).
    float score_candidate(const Eigen::Matrix4f& T_map_odom, int stride, float* median_out) const;

    struct Impl;
    std::unique_ptr<Impl> impl_;
    RelocConfig     cfg_;
    Eigen::Matrix4f T_map_odom_ = Eigen::Matrix4f::Identity();
    bool            seeded_ = false;
    float           seed_yaw_ = 0.f;   // heading of the seed — the yaw gate's reference
    std::deque<float> recent_best_yaws_;   // recent global_init best yaws (deg) — for stability accept
};

}  // namespace kist
