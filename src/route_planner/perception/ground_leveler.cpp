#include "route_planner/perception/ground_leveler.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace kist {

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;

// PCA plane normal of a point set: smallest-eigenvalue eigenvector of the covariance.
Eigen::Vector3d pca_normal(const std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d& centroid) {
    centroid.setZero();
    for (const auto& p : pts) centroid += p;
    centroid /= double(pts.size());
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : pts) { const Eigen::Vector3d d = p - centroid; cov += d * d.transpose(); }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    return es.eigenvectors().col(0).normalized();   // smallest eigenvalue -> plane normal
}
}  // namespace

double GroundLeveler::tilt_deg() const {
    const Eigen::Vector3d n = (accepted_ > 0) ? normal_sum_.normalized() : Eigen::Vector3d(0, 0, 1);
    return std::acos(std::clamp(n.z(), -1.0, 1.0)) / kDeg2Rad;
}

void GroundLeveler::reset() {
    T_leveled_odom_ = Transform{};
    locked_ = false; accepted_ = 0;
    normal_sum_.setZero(); centroid_sum_.setZero();
}

bool GroundLeveler::fit_floor(const LioCloud& scan, const Transform& T_odom_lidar,
                              const Transform& T_odom_pelvis,
                              Eigen::Vector3d& n_out, Eigen::Vector3d& c_out) const {
    const double sx = T_odom_lidar.translation.x();
    const double sy = T_odom_lidar.translation.y();
    const double floor_guess = T_odom_pelvis.translation.z() - cfg_.stand_height_m;
    const double range2 = double(cfg_.cand_range_m) * cfg_.cand_range_m;

    // Gather floor candidates: near the sensor in xy, near the expected floor height.
    std::vector<Eigen::Vector3d> cand;
    const std::size_t np = scan.point_count();
    const int step = std::max(1, cfg_.subsample);
    cand.reserve(np / step + 1);
    for (std::size_t i = 0; i < np; i += std::size_t(step)) {
        const double x = scan.xyz[3*i], y = scan.xyz[3*i+1], z = scan.xyz[3*i+2];
        if (std::abs(z - floor_guess) > cfg_.cand_band_m) continue;
        const double dx = x - sx, dy = y - sy;
        if (dx*dx + dy*dy > range2) continue;
        cand.emplace_back(x, y, z);
    }
    if (cand.size() < 20) return false;

    // RANSAC: best plane by inlier count (normal kept ~up).
    std::mt19937 rng(12345);
    std::uniform_int_distribution<std::size_t> pick(0, cand.size() - 1);
    const double thr = cfg_.ransac_inlier_m;
    int best_inliers = 0; Eigen::Vector3d best_n(0,0,1); double best_d = -floor_guess;
    for (int it = 0; it < cfg_.ransac_iters; ++it) {
        const Eigen::Vector3d& a = cand[pick(rng)];
        const Eigen::Vector3d& b = cand[pick(rng)];
        const Eigen::Vector3d& c = cand[pick(rng)];
        Eigen::Vector3d n = (b - a).cross(c - a);
        const double nn = n.norm();
        if (nn < 1e-6) continue;
        n /= nn;
        if (n.z() < 0) n = -n;                       // orient up
        const double d = -n.dot(a);
        int inl = 0;
        for (const auto& p : cand) if (std::abs(n.dot(p) + d) < thr) ++inl;
        if (inl > best_inliers) { best_inliers = inl; best_n = n; best_d = d; }
    }
    if (double(best_inliers) / double(cand.size()) < cfg_.min_inlier_ratio) return false;

    // Refine on the inlier set (PCA normal), and gate on plausibility (~vertical).
    std::vector<Eigen::Vector3d> inliers;
    inliers.reserve(best_inliers);
    for (const auto& p : cand) if (std::abs(best_n.dot(p) + best_d) < thr) inliers.push_back(p);
    Eigen::Vector3d centroid;
    Eigen::Vector3d n = pca_normal(inliers, centroid);
    if (n.z() < 0) n = -n;                            // orient up
    const double tilt = std::acos(std::clamp(n.z(), -1.0, 1.0));   // angle from +z
    if (tilt > cfg_.max_tilt_deg * kDeg2Rad) return false;

    n_out = n; c_out = centroid;
    return true;
}

void GroundLeveler::build_transform(const Eigen::Vector3d& n_in, const Eigen::Vector3d& c) {
    Eigen::Vector3d n = n_in.normalized();
    if (n.z() < 0) n = -n;
    // x = odom +X projected onto the floor plane (preserve heading); fall back to +Y if degenerate.
    Eigen::Vector3d ex(1, 0, 0);
    Eigen::Vector3d u = ex - (ex.dot(n)) * n;
    if (u.norm() < 1e-4) { ex = Eigen::Vector3d(0, 1, 0); u = ex - (ex.dot(n)) * n; }
    u.normalize();
    const Eigen::Vector3d v = n.cross(u);            // u x v = n (right-handed)

    Eigen::Matrix3d M;                               // p_level = M * p_odom + t
    M.row(0) = u; M.row(1) = v; M.row(2) = n;
    T_leveled_odom_.rotation    = Eigen::Quaterniond(M).normalized();
    T_leveled_odom_.translation = Eigen::Vector3d(0.0, 0.0, -n.dot(c));   // floor -> z = 0
}

bool GroundLeveler::update(const LioCloud& scan, const Transform& T_odom_lidar,
                           const Transform& T_odom_pelvis) {
    if (locked_) return false;

    Eigen::Vector3d n, c;
    if (!fit_floor(scan, T_odom_lidar, T_odom_pelvis, n, c)) return false;

    // Cross-frame agreement: reject a fit that disagrees with the running mean normal.
    if (accepted_ > 0) {
        const Eigen::Vector3d mean = normal_sum_.normalized();
        const double dtheta = std::acos(std::clamp(n.dot(mean), -1.0, 1.0));
        if (dtheta > cfg_.max_frame_dtheta_deg * kDeg2Rad) return false;
    }

    normal_sum_ += n; centroid_sum_ += c; ++accepted_;

    if (accepted_ >= cfg_.lock_frames) {
        build_transform(normal_sum_.normalized(), centroid_sum_ / double(accepted_));
        locked_ = true;
        return true;
    }
    return false;   // stay IDENTITY until locked (current behavior during startup)
}

} // namespace kist
