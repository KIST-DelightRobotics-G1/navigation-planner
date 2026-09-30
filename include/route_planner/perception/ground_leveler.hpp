#pragma once

// GroundLeveler — estimates T_leveled_odom, the per-boot, WORLD-FIXED transform that levels the
// odom frame to gravity so the floor becomes the XY plane. Pure logic (no thread / DDS); the
// PerceptionWorker feeds it each scan during startup.
//
// Why: the LIO odom (camera_init) is fixed at boot to the initial head/IMU orientation, which is
// NOT gravity-aligned — a tilted head at boot tilts the whole odom frame, so the floor rises into
// the obstacle band at range (false obstacles) and tall points smear in xy. LIO already compensates
// the DYNAMIC head sway into odom, so the odom<->gravity tilt is ~constant per run: estimate it once
// at startup (average N accepted floor fits), LOCK, and hold it for the rest of the run.
//
// Source: a RANSAC floor-plane fit on the registered scan (self-contained; no IMU wiring / map lock
// needed). Gated on normal plausibility (~vertical) + inlier ratio + agreement across frames.
// Before LOCK (or if fits are rejected) T_leveled_odom is IDENTITY — i.e. exactly today's behavior.
//
// Frame construction (Gram-Schmidt, preserves odom heading): z = floor normal (up), x = odom +X
// projected onto the floor plane, y = z x x; translation puts the floor at z = 0. So a point
// p_level = T_leveled_odom.transformPoint(p_odom) has p_level.z = height above the floor.

#include "lio/lio_cloud.hpp"
#include "transforms/transform.hpp"

#include <Eigen/Dense>
#include <cstdint>

namespace kist {

struct GroundLevelerConfig {
    int   lock_frames        = 20;     // accepted floor fits to average before LOCK
    int   ransac_iters       = 120;    // RANSAC trials per scan
    float ransac_inlier_m    = 0.05f;  // inlier band (m) around the candidate plane
    int   subsample          = 4;      // use every Nth cloud point
    float cand_range_m       = 6.0f;   // floor candidate: within this xy range of the sensor
    float cand_band_m        = 0.30f;  // floor candidate: within this of the expected floor height
    float stand_height_m     = 0.89f;  // pelvis height above floor (expected-floor seed)
    float max_tilt_deg       = 20.0f;  // reject a plane whose normal tilts more than this from up
    float min_inlier_ratio   = 0.40f;  // reject a fit with fewer inliers than this
    float max_frame_dtheta_deg = 8.0f; // reject a frame whose normal disagrees with the running mean
};

class GroundLeveler {
public:
    void set_config(const GroundLevelerConfig& c) { cfg_ = c; }
    const GroundLevelerConfig& config() const { return cfg_; }

    // Feed one registered scan (odom frame) + the current sensor / base poses. During startup this
    // fits + averages the floor plane; once locked it is a cheap no-op. Returns true if LOCKED now.
    bool update(const LioCloud& scan_odom, const Transform& T_odom_lidar,
                const Transform& T_odom_pelvis);

    bool             locked() const { return locked_; }
    const Transform& T_leveled_odom() const { return T_leveled_odom_; }  // identity until locked
    int              accepted_frames() const { return accepted_; }
    double           tilt_deg() const;   // current estimated floor tilt from up (deg); 0 before a fit

    void reset();   // drop the lock (e.g. on a re-init) — back to identity + re-estimate

private:
    // One RANSAC floor fit on the scan; false if no plausible floor found. Fills n (unit, up) + c
    // (a floor inlier centroid), both in odom.
    bool fit_floor(const LioCloud& scan_odom, const Transform& T_odom_lidar,
                   const Transform& T_odom_pelvis, Eigen::Vector3d& n, Eigen::Vector3d& c) const;

    // Build T_leveled_odom from the averaged normal + floor centroid.
    void build_transform(const Eigen::Vector3d& n, const Eigen::Vector3d& c);

    GroundLevelerConfig cfg_;
    Transform           T_leveled_odom_;               // identity by default (fallback)
    bool                locked_ = false;
    int                 accepted_ = 0;
    Eigen::Vector3d     normal_sum_   = Eigen::Vector3d::Zero();
    Eigen::Vector3d     centroid_sum_ = Eigen::Vector3d::Zero();
};

} // namespace kist
