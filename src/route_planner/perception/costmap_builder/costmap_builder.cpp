#include "route_planner/perception/costmap_builder/costmap_builder.hpp"

#include "route_planner/perception/costmap_builder/edt.hpp"

#include <algorithm>
#include <cmath>

namespace kist {

// Cost as a function of distance to the nearest obstacle, precomputed per squared-distance
// (cell units) so build() is a table lookup.
void CostmapBuilder::build_lut(float resolution, const CostmapConfig& cfg) {
    lut_res_ = resolution;
    const int max_cells = int(std::ceil(cfg.influence_radius_m / resolution));
    const int max_sq    = (max_cells + 1) * (max_cells + 1);
    lut_.resize(std::size_t(max_sq) + 1);
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
        lut_[std::size_t(d2)] = cost;
    }
}

Costmap CostmapBuilder::build(const ObstacleGrid& g, const ObstacleGridConfig& gcfg,
                              const CostmapConfig& cfg) {
    Costmap cm;
    if (g.empty()) return cm;

    cm.n = g.n;  cm.resolution = g.resolution;
    cm.origin_x = g.origin_x;  cm.origin_y = g.origin_y;
    cm.robot_x = g.robot_x;  cm.robot_y = g.robot_y;  cm.robot_yaw = g.robot_yaw;
    cm.stamp_ns = g.stamp_ns;

    // Obstacle mask: occupied cells (P > occ_threshold), row-major to match the EDT.
    const int N = g.n * g.n;
    std::vector<uint8_t> obst(std::size_t(N), 0);
    for (int i = 0; i < N; ++i)
        if (g.prob(i) > gcfg.occ_threshold) obst[std::size_t(i)] = 1;

    // Noise removal: drop occupied cells with too few occupied neighbours — isolated
    // returns each inflate into a lethal disk and wall off traversable space, while real
    // walls are dense and survive. Neighbour counts use the pre-filter snapshot.
    if (cfg.obstacle_min_neighbors > 0) {
        const std::vector<uint8_t> src = obst;
        for (int iy = 0; iy < g.n; ++iy)
            for (int ix = 0; ix < g.n; ++ix) {
                if (!src[std::size_t(iy) * g.n + ix]) continue;
                int cnt = 0;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (!dx && !dy) continue;
                        const int nx = ix + dx, ny = iy + dy;
                        if (nx < 0 || nx >= g.n || ny < 0 || ny >= g.n) continue;
                        cnt += src[std::size_t(ny) * g.n + nx];
                    }
                if (cnt < cfg.obstacle_min_neighbors) obst[std::size_t(iy) * g.n + ix] = 0;
            }
    }

    if (lut_.empty() || lut_res_ != g.resolution)
        build_lut(g.resolution, cfg);   // cfg is constant per run

    const std::vector<float> d2 = edt_squared(obst, g.n, g.n);
    const int lut_max = int(lut_.size()) - 1;
    cm.cells.resize(std::size_t(N));
    for (int i = 0; i < N; ++i) {
        const int q = int(std::min(d2[std::size_t(i)], float(lut_max)));
        cm.cells[std::size_t(i)] = lut_[std::size_t(q)];
    }
    return cm;
}

} // namespace kist
