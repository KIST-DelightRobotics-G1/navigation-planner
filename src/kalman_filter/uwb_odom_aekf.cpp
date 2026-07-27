#include "kalman_filter/uwb_odom_aekf.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

namespace kist {

namespace {

// Wrap an angle (rad) to [-pi, pi] — matches the Python (a+pi)%(2pi)-pi.
double wrap_rad(double a) {
    double m = std::fmod(a + M_PI, 2.0 * M_PI);
    if (m < 0.0) m += 2.0 * M_PI;
    return m - M_PI;
}

} // namespace

UwbOdomAEKFParams uwb_odom_aekf_params_from_yaml(const YAML::Node& section) {
    UwbOdomAEKFParams p;
    if (!section) return p;  // absent section -> all defaults
    p.q_xy_m2_per_m            = section["q_xy_m2_per_m"].as<double>(p.q_xy_m2_per_m);
    p.q_bias_rad2_per_step     = section["q_bias_rad2_per_step"].as<double>(p.q_bias_rad2_per_step);
    p.p_init_xy_m2             = section["p_init_xy_m2"].as<double>(p.p_init_xy_m2);
    p.p_init_bias_rad2         = section["p_init_bias_rad2"].as<double>(p.p_init_bias_rad2);
    p.r_init_m2                = section["r_init_m2"].as<double>(p.r_init_m2);
    p.r_min_m2                 = section["r_min_m2"].as<double>(p.r_min_m2);
    p.r_max_m2                 = section["r_max_m2"].as<double>(p.r_max_m2);
    p.forget_b                 = section["forget_b"].as<double>(p.forget_b);
    p.warmup_steps             = section["warmup_steps"].as<int>(p.warmup_steps);
    p.bias_calibrated_std_deg  = section["bias_calibrated_std_deg"].as<double>(p.bias_calibrated_std_deg);
    return p;
}

UwbOdomAEKF::UwbOdomAEKF(UwbOdomAEKFParams params) : p_(params) { reset(); }

void UwbOdomAEKF::initialize(double x_m, double y_m, double odom_yaw_rad) {
    x_ << x_m, y_m, 0.0;
    P_ = Eigen::Vector3d(p_.p_init_xy_m2, p_.p_init_xy_m2, p_.p_init_bias_rad2).asDiagonal();
    R_ = Eigen::Vector2d(p_.r_init_m2, p_.r_init_m2).asDiagonal();
    n_updates_    = 0;
    sh_step_      = 0;
    prev_yaw_cal_ = false;
    has_prev_odom_ = false;
    has_prev_uwb_  = false;
    last_odom_yaw_ = odom_yaw_rad;
    initialized_   = true;
}

void UwbOdomAEKF::reset() {
    x_.setZero();
    P_ = Eigen::Vector3d(p_.p_init_xy_m2, p_.p_init_xy_m2, p_.p_init_bias_rad2).asDiagonal();
    R_ = Eigen::Vector2d(p_.r_init_m2, p_.r_init_m2).asDiagonal();
    initialized_  = false;
    n_updates_    = 0;
    sh_step_      = 0;
    prev_yaw_cal_ = false;
    has_prev_odom_ = false;
    has_prev_uwb_  = false;
    last_odom_yaw_ = 0.0;
}

void UwbOdomAEKF::predict(double odom_x_m, double odom_y_m, double odom_yaw_rad) {
    last_odom_yaw_ = odom_yaw_rad;

    if (!has_prev_odom_) {
        prev_odom_x_ = odom_x_m;
        prev_odom_y_ = odom_y_m;
        has_prev_odom_ = true;
        return;
    }

    const double dx = odom_x_m - prev_odom_x_;
    const double dy = odom_y_m - prev_odom_y_;
    const double delta_s = std::sqrt(dx * dx + dy * dy);
    prev_odom_x_ = odom_x_m;
    prev_odom_y_ = odom_y_m;

    if (!initialized_) return;

    const double b = x_(2);
    const double cb = std::cos(b);
    const double sb = std::sin(b);

    x_(0) += dx * cb - dy * sb;
    x_(1) += dx * sb + dy * cb;
    // x_(2) unchanged (constant model)

    Eigen::Matrix3d F;
    F << 1.0, 0.0, -dx * sb - dy * cb,
         0.0, 1.0,  dx * cb - dy * sb,
         0.0, 0.0,  1.0;

    const double abs_s = std::max(delta_s, 1e-6);
    Eigen::Matrix3d Q = Eigen::Vector3d(p_.q_xy_m2_per_m * abs_s,
                                        p_.q_xy_m2_per_m * abs_s,
                                        p_.q_bias_rad2_per_step).asDiagonal();

    P_ = F * P_ * F.transpose() + Q;
}

void UwbOdomAEKF::reset_odom_reference() {
    has_prev_odom_ = false;
}

bool UwbOdomAEKF::update(double uwb_x_m, double uwb_y_m) {
    if (!initialized_) return false;

    if (has_prev_uwb_ && uwb_x_m == prev_uwb_x_ && uwb_y_m == prev_uwb_y_)
        return false;

    prev_uwb_x_ = uwb_x_m;
    prev_uwb_y_ = uwb_y_m;
    has_prev_uwb_ = true;
    return do_update(uwb_x_m, uwb_y_m);
}

bool UwbOdomAEKF::do_update(double meas_x_m, double meas_y_m) {
    Eigen::Matrix<double, 2, 3> H;
    H << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0;

    const Eigen::Vector2d z(meas_x_m, meas_y_m);
    const Eigen::Vector2d innov = z - x_.head<2>();
    const Eigen::Matrix3d P_pred = P_;

    // Sage-Husa adaptive R — enabled only after yaw calibration.
    const bool cal_now = yaw_calibrated();
    if (cal_now && !prev_yaw_cal_) sh_step_ = 0;
    prev_yaw_cal_ = cal_now;

    if (n_updates_ >= p_.warmup_steps && cal_now) {
        const double d_k = (1.0 - p_.forget_b) /
                           (1.0 - std::pow(p_.forget_b, sh_step_ + 1));
        const Eigen::Matrix2d R_innov =
            innov * innov.transpose() - H * P_pred * H.transpose();
        R_ = (1.0 - d_k) * R_ + d_k * R_innov;
        // Python collapses R back to a clamped diagonal each step.
        const double r0 = std::clamp(R_(0, 0), p_.r_min_m2, p_.r_max_m2);
        const double r1 = std::clamp(R_(1, 1), p_.r_min_m2, p_.r_max_m2);
        R_ = Eigen::Vector2d(r0, r1).asDiagonal();
        ++sh_step_;
    }

    const Eigen::Matrix2d S = H * P_pred * H.transpose() + R_;
    // K = (S^-1 (H P_pred))^T, shape 3x2 (matches np.linalg.solve(S, H@P).T).
    const Eigen::Matrix<double, 3, 2> K =
        S.ldlt().solve(H * P_pred).transpose();

    x_ = x_ + K * innov;
    x_(2) = wrap_rad(x_(2));

    const Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
    P_ = I_KH * P_pred * I_KH.transpose() + K * R_ * K.transpose();

    ++n_updates_;
    return true;
}

double UwbOdomAEKF::global_yaw_rad() const {
    return wrap_rad(last_odom_yaw_ + x_(2));
}

double UwbOdomAEKF::std_xy_m() const {
    return std::sqrt((P_(0, 0) + P_(1, 1)) / 2.0);
}

double UwbOdomAEKF::std_bias_deg() const {
    return std::sqrt(std::max(0.0, P_(2, 2))) * 180.0 / M_PI;
}

bool UwbOdomAEKF::yaw_calibrated() const {
    const double threshold_rad = p_.bias_calibrated_std_deg * M_PI / 180.0;
    return P_(2, 2) < threshold_rad * threshold_rad;
}

} // namespace kist
