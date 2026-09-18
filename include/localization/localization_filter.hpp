#pragma once

// LocalizationFilter — a tiny 3-state EKF that estimates T_map_odom (the low-frequency map<-odom
// GLOBAL CORRECTION), fusing the GICP geometric constraint with the UWB position constraint. FAST-LIO
// is NOT re-filtered here; T_odom_base stays the engine's smooth high-rate odometry — we only filter
// the slow correction on top of it.
//
// State  x = [px, py, yaw]  (SE(2) of T_map_odom). z comes from GICP (kept aside); roll/pitch = 0
//        (map and odom are both gravity-aligned).
// Predict: random walk   x_k = x_{k-1} + w,   P += Q.
// Measurements:
//   GICP : raw T_map_odom (H = I, 3-DoF).
//   UWB  : the antenna xy in map -> nonlinear in yaw (2x3 Jacobian).
// Both are Mahalanobis-gated; an extreme UWB disagreement flags LOST (caller re-runs global_init).
//
// Pure logic — no thread, no DDS, no PCL. Driven by the Localization worker each cycle:
//   predict() -> update_gicp(T_gicp) -> update_uwb(p_uwb, T_odom_base, t_base_uwb) -> T_map_odom().

#include <Eigen/Dense>

namespace kist {

struct LocFilterConfig {
    // Process noise per predict step (~1 Hz). map<-odom drifts slowly, so keep small.
    double q_xy_m    = 0.05;   // position random-walk std per cycle (m)
    double q_yaw_deg = 1.0;    // yaw      random-walk std per cycle (deg)
    // Measurement noise (isotropic for now; GICP R can be made anisotropic from the Hessian later).
    double gicp_sigma_xy_m  = 0.10;
    double gicp_sigma_yaw_deg = 2.0;
    double uwb_sigma_xy_m   = 0.15;   // UWB self-gates to <30cm, so ~0.15 std is reasonable
    // Mahalanobis chi-square gates (reject a measurement whose normalized innovation exceeds this).
    double gicp_gate = 11.34;  // 3-DoF, ~99%
    double uwb_gate  = 9.21;   // 2-DoF, ~99%
    // Extreme UWB<->prediction disagreement (m): flag LOST so the caller re-initializes.
    double lost_uwb_m = 0.8;
    // Initial state covariance (from a fresh global_init seed).
    double init_sigma_xy_m  = 0.30;
    double init_sigma_yaw_deg = 5.0;
};

class LocalizationFilter {
public:
    void set_config(const LocFilterConfig& c) { cfg_ = c; }
    const LocFilterConfig& config() const { return cfg_; }

    // Initialize the state from a global_init result (T_map_odom). Resets covariance to init_*.
    void seed(const Eigen::Matrix4f& T_map_odom);
    bool seeded() const { return seeded_; }
    void reset() { seeded_ = false; }   // drop the lock (LOST) — caller re-runs global_init

    // Random-walk prediction; call once per cycle before the updates.
    void predict();

    // GICP measurement: the raw T_map_odom from GICP. Returns false if Mahalanobis-rejected
    // (state left unchanged). z is stored for the output pose.
    bool update_gicp(const Eigen::Matrix4f& T_gicp);

    // UWB measurement: p_uwb = antenna xy in map; T_odom_base = current LIO base pose (odom<-base);
    // t_base_uwb = antenna offset in the base frame. Returns false if rejected. If *lost is set true,
    // the UWB is wildly off the prediction -> the caller should re-run global_init.
    bool update_uwb(const Eigen::Vector2d& p_uwb, const Eigen::Matrix4f& T_odom_base,
                    const Eigen::Vector3d& t_base_uwb, bool* lost = nullptr);

    // Current estimate as a 4x4 (Rz(yaw), translation (px,py,z)); roll/pitch = 0.
    Eigen::Matrix4f T_map_odom() const;

    Eigen::Vector3d state() const { return x_; }          // [px, py, yaw(rad)]
    Eigen::Matrix3d covariance() const { return P_; }

private:
    Eigen::Vector3d x_ = Eigen::Vector3d::Zero();          // [px, py, yaw(rad)]
    Eigen::Matrix3d P_ = Eigen::Matrix3d::Identity();
    double          z_ = 0.0;                               // map<-odom z, carried from GICP
    bool            seeded_ = false;
    LocFilterConfig cfg_;
};

} // namespace kist
