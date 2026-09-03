#pragma once

#include <Eigen/Dense>

#include <cmath>

namespace YAML {
class Node;
}

namespace kist {

// Tuning knobs for UwbOdomAEKF. Defaults match the onboard Python reference;
// override per deployment from the `pose_filter` section of config.yaml.
struct UwbOdomAEKFParams {
    // Process noise (Q)
    double q_xy_m2_per_m        = 0.001;   // x/y process noise (m^2/m), std ~3cm/m
    double q_bias_rad2_per_step = 1e-9;    // b_theta random walk, std ~0.0018deg/step
    // Initial covariance (P)
    double p_init_xy_m2   = 1.0;
    double p_init_bias_rad2 = M_PI * M_PI; // b_theta fully unknown at init
    // Adaptive R (Sage-Husa)
    double r_init_m2 = 0.25;
    double r_min_m2  = 0.002;
    double r_max_m2  = 25.0;
    double forget_b  = 0.98;
    int    warmup_steps = 20;
    // Yaw calibration threshold: b_theta std below this (deg) -> calibrated
    double bias_calibrated_std_deg = 5.0;
};

// Reads the `pose_filter` section of config.yaml (pass
// Config::instance().root()["pose_filter"]). Missing keys keep their
// defaults, so a partial section (or an absent one) is fine.
UwbOdomAEKFParams uwb_odom_aekf_params_from_yaml(const YAML::Node& section);

// UWB + odometry fusion via an Adaptive EKF with online yaw-bias estimation
// (indoor, holonomic). A faithful C++/Eigen port of the onboard Python
// UwbOdomAEKF (kist-drl-g1-onboard sensors/utils/uwb_odom_aekf.py) — the
// algorithm is copied, the code is not.
//
// This is a pure Module: no threads, no I/O, no timestamps. PoseFilter's
// worker thread is its single driver (one writer), so it needs no locking.
//
//   State  : [x, y, b_theta]      (UWB local frame)
//              x, y     UWB-frame position (m)
//              b_theta  rotation offset aligning the odom position frame's
//                       x/y axes onto the UWB frame's (rad)
//   Process: rotate the odom displacement by b_theta into the UWB frame
//              [dx_uwb, dy_uwb]^T = R(b_theta) . [dx_odom, dy_odom]^T
//            b_theta is near-constant (slow random walk).
//   Measure: UWB x/y directly. H's b_theta column is 0, so UWB never observes
//            b_theta directly — but motion couples it through F, so the
//            position innovation updates b_theta indirectly. It only converges
//            while moving; yaw_calibrated() reports when its std has shrunk
//            below threshold. R is estimated online (Sage-Husa), enabled only
//            after calibration to avoid a positive-feedback loop.
class UwbOdomAEKF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit UwbOdomAEKF(UwbOdomAEKFParams params = {});

    // Seed the filter at a UWB fix. b_theta starts at 0 with full (pi^2)
    // uncertainty and converges from motion. odom_yaw feeds global_yaw output.
    // A remembered yaw bias can be seeded instead: pass b_theta_rad plus its
    // variance (rad^2) — a small variance makes yaw_calibrated() true at once,
    // skipping the drive-to-calibrate step (only valid if odom is continuous).
    // b_theta_var_rad2 < 0 keeps the default full uncertainty.
    void initialize(double x_m, double y_m, double odom_yaw_rad = 0.0,
                    double b_theta_rad = 0.0, double b_theta_var_rad2 = -1.0);

    // Back to the pre-initialize state; predict/update are ignored until the
    // next initialize().
    void reset();

    // Time update from raw odometry coordinates (holonomic model). The first
    // call after initialize()/reset() only seeds the previous-odom reference.
    void predict(double odom_x_m, double odom_y_m, double odom_yaw_rad);

    // Drop the previous-odom reference so the next predict() re-seeds instead
    // of differencing across a gap. Call after an odometry stream dropout: the
    // estimate (x, P, b_theta) is kept, only the increment baseline is reset,
    // which avoids a bogus jump from a stale/discontinuous prev_odom.
    void reset_odom_reference();

    // Measurement update from a UWB fix. Returns false (no-op) before
    // initialize() or when the fix repeats the previous value (receiver freeze).
    bool update(double uwb_x_m, double uwb_y_m);

    // ── read-only state ────────────────────────────────────────
    double x_m()         const { return x_(0); }
    double y_m()         const { return x_(1); }
    double b_theta_rad() const { return x_(2); }
    double global_yaw_rad() const;              // wrap(last_odom_yaw + b_theta)
    double std_xy_m()    const;
    double std_bias_deg() const;
    bool   yaw_calibrated() const;              // b_theta std below threshold
    bool   initialized() const { return initialized_; }
    int    n_updates()   const { return n_updates_; }

private:
    bool do_update(double meas_x_m, double meas_y_m);

    UwbOdomAEKFParams p_;

    Eigen::Vector3d x_;      // [x, y, b_theta]
    Eigen::Matrix3d P_;
    Eigen::Matrix2d R_;

    bool   initialized_  = false;
    int    n_updates_    = 0;
    int    sh_step_      = 0;
    bool   prev_yaw_cal_ = false;

    bool   has_prev_odom_ = false;
    double prev_odom_x_   = 0.0;
    double prev_odom_y_   = 0.0;
    double last_odom_yaw_ = 0.0;

    bool   has_prev_uwb_ = false;
    double prev_uwb_x_   = 0.0;
    double prev_uwb_y_   = 0.0;
};

} // namespace kist
