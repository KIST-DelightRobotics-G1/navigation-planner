#include "costmap/costmap_builder.hpp"

#include "costmap/edt.hpp"

#include <opencv2/imgproc.hpp>   // dilate (dynamic inflation)

#include <algorithm>
#include <cmath>

namespace kist {

// Cost as a function of distance to the nearest obstacle, precomputed per
// squared-distance (cell units) so build() is a table lookup.
void CostmapBuilder::build_lut(float resolution, const CostmapConfig& cfg) {
    lut_res_ = resolution;
    const int max_cells = int(std::ceil(cfg.influence_radius_m / resolution));
    const int max_sq    = (max_cells + 1) * (max_cells + 1);
    lut_.resize(size_t(max_sq) + 1);
    for (int d2 = 0; d2 <= max_sq; ++d2) {
        const float d = std::sqrt(float(d2)) * resolution;   // metres
        uint8_t cost;
        if (d <= cfg.lethal_radius_m)          cost = cfg.lethal_cost;
        else if (d >= cfg.influence_radius_m)  cost = 0;
        else {
            const float raw = float(cfg.max_soft_cost) *
                              std::exp(-cfg.decay * (d - cfg.lethal_radius_m));
            cost = uint8_t(std::clamp(int(raw), 0, int(cfg.max_soft_cost)));
        }
        lut_[size_t(d2)] = cost;
    }
}

Costmap CostmapBuilder::build(const OccupancyGrid& g, const GridConfig& gcfg,
                              const CostmapConfig& cfg, const cv::Mat& dynamic_mask) {
    Costmap cm;
    if (g.empty()) return cm;
    const bool have_dyn_mask = cfg.ignore_dynamic && dynamic_mask.rows == g.n &&
                               dynamic_mask.cols == g.n;

    cm.n = g.n;  cm.resolution = g.resolution;
    cm.origin_x = g.origin_x;  cm.origin_y = g.origin_y;
    cm.robot_x = g.robot_x;  cm.robot_y = g.robot_y;  cm.robot_yaw = g.robot_yaw;
    cm.stamp_ns = g.stamp_ns;

    // Obstacle mask: occupied cells, row-major (iy*n + ix) to match the EDT.
    const int N = g.n * g.n;
    std::vector<uint8_t> obst(size_t(N), 0);
    cv::Mat dyn(g.n, g.n, CV_8U, cv::Scalar(0));   // dynamic obstacles only
    for (int iy = 0; iy < g.n; ++iy)
        for (int ix = 0; ix < g.n; ++ix) {
            const int gi = g.index(ix, iy);
            if (g.prob(gi) <= gcfg.occ_threshold) continue;
            const uint8_t lab = g.label[gi];
            const bool is_dyn = (lab != kNoClass && gcfg.is_dynamic[lab]);
            // People are not obstacles for the planner — the robot stops and asks
            // to pass rather than swerving. Ignore the whole person FOOTPRINT (the
            // cluster region, incl. absorbed grey cells) via the dynamic mask; fall
            // back to the per-cell label when no mask was supplied.
            if (cfg.ignore_dynamic &&
                (have_dyn_mask ? dynamic_mask.at<uint8_t>(iy, ix) != 0 : is_dyn))
                continue;
            obst[size_t(iy) * g.n + ix] = 1;
            if (is_dyn) dyn.at<uint8_t>(iy, ix) = 255;
        }

    // Semantic inflation: when people ARE treated as obstacles, grow them by an
    // extra margin. Skipped when ignore_dynamic (they're not in the map at all).
    const int extra = int(cfg.dynamic_inflation_m / g.resolution + 0.5f);
    if (!cfg.ignore_dynamic && extra > 0 && cv::countNonZero(dyn) > 0) {
        cv::dilate(dyn, dyn, cv::getStructuringElement(cv::MORPH_ELLIPSE, {2*extra+1, 2*extra+1}));
        for (int iy = 0; iy < g.n; ++iy)
            for (int ix = 0; ix < g.n; ++ix)
                if (dyn.at<uint8_t>(iy, ix)) obst[size_t(iy) * g.n + ix] = 1;
    }

    if (lut_.empty() || lut_res_ != g.resolution)
        build_lut(g.resolution, cfg);   // cfg is constant per run

    const std::vector<float> d2 = edt_squared(obst, g.n, g.n);
    const int lut_max = int(lut_.size()) - 1;
    cm.cells.resize(size_t(N));
    for (int i = 0; i < N; ++i) {
        const int q = int(std::min(d2[size_t(i)], float(lut_max)));
        cm.cells[size_t(i)] = lut_[size_t(q)];
    }
    return cm;
}

} // namespace kist
