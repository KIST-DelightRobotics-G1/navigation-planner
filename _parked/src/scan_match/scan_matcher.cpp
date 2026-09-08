#include "scan_match/scan_matcher.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>

namespace kist {

Pose2D scan_match(const OccupancyGrid& g, const GridConfig& gcfg,
                  const std::vector<float>& scan_xy,
                  const Pose2D& prior, const ScanMatchConfig& cfg) {
    if (!cfg.enabled || g.empty()) return prior;
    const size_t n = scan_xy.size() / 2;
    if (int(n) < cfg.min_scan_points) return prior;

    // Occupied mask (0 = occupied, 255 = free) for the distance transform.
    cv::Mat occ(g.n, g.n, CV_8U, cv::Scalar(255));
    int occ_count = 0;
    for (int iy = 0; iy < g.n; ++iy)
        for (int ix = 0; ix < g.n; ++ix)
            if (g.prob(g.index(ix, iy)) > gcfg.occ_threshold) {
                occ.at<uint8_t>(iy, ix) = 0;
                ++occ_count;
            }
    if (occ_count < cfg.min_map_cells) return prior;

    // Likelihood field: exp(-d^2 / 2 sigma^2), d = distance (cells) to nearest occupied.
    cv::Mat dist;
    cv::distanceTransform(occ, dist, cv::DIST_L2, 3);
    const float sig_cells = std::max(1e-3f, cfg.sigma_m / g.resolution);
    const float inv2s2    = 1.0f / (2.0f * sig_cells * sig_cells);
    cv::Mat lik;
    cv::multiply(dist, dist, lik);          // d^2
    lik *= -inv2s2;
    cv::exp(lik, lik);                       // exp(-d^2/2s^2), CV_32F

    auto score = [&](float x, float y, float c, float s) -> float {
        float sum = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            const float rx = scan_xy[2*i], ry = scan_xy[2*i+1];
            const float wx = x + c*rx - s*ry, wy = y + s*rx + c*ry;
            int ix, iy;
            if (g.world_to_cell(wx, wy, ix, iy)) sum += lik.at<float>(iy, ix);
        }
        return sum;
    };

    // Correlative search over the window around the prior. Yaw candidates are
    // precomputed (cos/sin) since translation only shifts the lookup.
    const int   nxy    = int(cfg.search_xy_m / cfg.step_xy_m);
    const float yaw_hw = cfg.search_yaw_deg * float(M_PI) / 180.0f;
    const int   ny     = std::max(1, cfg.yaw_steps);

    Pose2D best   = prior;
    float  best_s = score(prior.x, prior.y, std::cos(prior.yaw), std::sin(prior.yaw));

    for (int k = 0; k < ny; ++k) {
        const float dyaw = (ny > 1) ? (-yaw_hw + 2.0f * yaw_hw * k / (ny - 1)) : 0.0f;
        const float yaw  = prior.yaw + dyaw;
        const float c = std::cos(yaw), s = std::sin(yaw);
        for (int jy = -nxy; jy <= nxy; ++jy)
            for (int jx = -nxy; jx <= nxy; ++jx) {
                const float x = prior.x + jx * cfg.step_xy_m;
                const float y = prior.y + jy * cfg.step_xy_m;
                const float sc = score(x, y, c, s);
                if (sc > best_s) { best_s = sc; best = {x, y, yaw}; }
            }
    }
    return best;
}

} // namespace kist
