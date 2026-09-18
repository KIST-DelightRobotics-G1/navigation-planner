#include "localization/localization_filter.hpp"

#include <cmath>

namespace kist {

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;

double wrap_pi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
double yaw_of(const Eigen::Matrix4f& T) { return std::atan2(double(T(1, 0)), double(T(0, 0))); }
}  // namespace

void LocalizationFilter::seed(const Eigen::Matrix4f& T) {
    x_ << double(T(0, 3)), double(T(1, 3)), yaw_of(T);
    z_ = double(T(2, 3));
    const double sxy = cfg_.init_sigma_xy_m, syaw = cfg_.init_sigma_yaw_deg * kDeg2Rad;
    P_ = Eigen::Vector3d(sxy*sxy, sxy*sxy, syaw*syaw).asDiagonal();
    seeded_ = true;
}

void LocalizationFilter::predict(double dt) {
    if (!seeded_) return;
    if (dt < 0.0) dt = 0.0;
    const double qxy = cfg_.q_xy_m, qyaw = cfg_.q_yaw_deg * kDeg2Rad;
    P_(0, 0) += qxy * qxy * dt;                 // variance grows with elapsed time (random walk)
    P_(1, 1) += qxy * qxy * dt;
    P_(2, 2) += qyaw * qyaw * dt;
    // x_ unchanged (random walk).
}

bool LocalizationFilter::update_gicp(const Eigen::Matrix4f& T_gicp) {
    if (!seeded_) { seed(T_gicp); return true; }

    Eigen::Vector3d zmeas(double(T_gicp(0, 3)), double(T_gicp(1, 3)), yaw_of(T_gicp));
    Eigen::Vector3d y = zmeas - x_;
    y(2) = wrap_pi(y(2));                                   // yaw innovation wrapped

    const double sxy = cfg_.gicp_sigma_xy_m, syaw = cfg_.gicp_sigma_yaw_deg * kDeg2Rad;
    Eigen::Matrix3d R = Eigen::Vector3d(sxy*sxy, sxy*sxy, syaw*syaw).asDiagonal();
    Eigen::Matrix3d S = P_ + R;                             // H = I
    const double d2 = y.dot(S.ldlt().solve(y));             // Mahalanobis
    if (d2 > cfg_.gicp_gate) return false;                  // reject outlier / wrong mode

    Eigen::Matrix3d K = P_ * S.inverse();
    x_ += K * y;
    x_(2) = wrap_pi(x_(2));
    P_ = (Eigen::Matrix3d::Identity() - K) * P_;
    z_ = double(T_gicp(2, 3));                              // carry z from GICP
    return true;
}

bool LocalizationFilter::update_uwb(const Eigen::Vector2d& p_uwb, const Eigen::Matrix4f& T_odom_base,
                                    const Eigen::Vector3d& t_base_uwb, bool* lost) {
    if (lost) *lost = false;
    if (!seeded_) return false;

    // Antenna position in the ODOM frame: a = T_odom_base * t_base_uwb.
    const Eigen::Matrix3d Rob = T_odom_base.block<3,3>(0,0).cast<double>();
    const Eigen::Vector3d tob = T_odom_base.block<3,1>(0,3).cast<double>();
    const Eigen::Vector3d a3  = Rob * t_base_uwb + tob;
    const double ax = a3.x(), ay = a3.y();

    const double th = x_(2), c = std::cos(th), s = std::sin(th);
    // Predicted antenna xy in map: p = [px + c*ax - s*ay, py + s*ax + c*ay].
    Eigen::Vector2d p_pred(x_(0) + c*ax - s*ay, x_(1) + s*ax + c*ay);
    Eigen::Vector2d y = p_uwb - p_pred;

    if (y.norm() > cfg_.lost_uwb_m) { if (lost) *lost = true; return false; }  // extreme -> LOST

    // Jacobian H = d p_pred / d[px,py,yaw]  (2x3).
    Eigen::Matrix<double, 2, 3> H;
    H << 1.0, 0.0, (-s*ax - c*ay),
         0.0, 1.0, ( c*ax - s*ay);

    const double su = cfg_.uwb_sigma_xy_m;
    Eigen::Matrix2d R = Eigen::Vector2d(su*su, su*su).asDiagonal();
    Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    const double d2 = y.dot(S.ldlt().solve(y));
    if (d2 > cfg_.uwb_gate) return false;                   // reject (NLOS spike, etc.)

    Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();
    x_ += K * y;
    x_(2) = wrap_pi(x_(2));
    P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
    return true;
}

Eigen::Matrix4f LocalizationFilter::T_map_odom() const {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    const float c = float(std::cos(x_(2))), s = float(std::sin(x_(2)));
    T(0, 0) = c; T(0, 1) = -s;
    T(1, 0) = s; T(1, 1) =  c;
    T(0, 3) = float(x_(0));
    T(1, 3) = float(x_(1));
    T(2, 3) = float(z_);
    return T;
}

} // namespace kist
