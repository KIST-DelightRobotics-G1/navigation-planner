#pragma once

// Ground-plane estimation from the (robot-frame) camera cloud. The head cam is
// tilted down and sees the floor as a large dense patch, so RANSAC — constrained
// to near-horizontal planes near the floor — fits it robustly. Its normal gives
// the true sensor-to-floor tilt (which the base odom under-reports, because the
// head mount bobs more than the pelvis), used downstream to level the clouds.

#include <cstddef>
#include <vector>

namespace kist {

struct GroundPlane {
    float nx = 0.f, ny = 0.f, nz = 1.f;   // unit normal (robot frame), points up
    float d  = 0.f;                        // plane: nx*x + ny*y + nz*z + d = 0
    int   inliers = 0;
    bool  valid = false;
};

struct GroundPlaneConfig {
    bool  enabled        = true;
    float inlier_eps_m   = 0.05f;   // point-plane distance counted as ground
    float max_tilt_deg   = 30.0f;   // reject a plane whose normal tilts > this from up
    float candidate_z_max= 0.6f;    // only points below this (robot z) seed the fit
    int   iterations     = 120;     // RANSAC trials
    int   min_inliers    = 300;     // below this the fit is rejected (fallback)
    float smooth         = 0.5f;    // temporal blend with the previous normal (0..1; 1 = none)
};

// Fit the ground plane from robot-frame points (xyz packed [x,y,z,...]). `prev`
// (if valid) seeds temporal smoothing and is returned on failure so the estimate
// degrades gracefully rather than flickering to identity.
GroundPlane fit_ground_plane(const std::vector<float>& xyz,
                             const GroundPlaneConfig& cfg,
                             const GroundPlane& prev = {});

} // namespace kist
