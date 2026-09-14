#include "route_planner/perception/costmap_builder/clearance.hpp"

#include "route_planner/perception/costmap_builder/edt.hpp"

#include <cmath>

namespace kist {

std::vector<float> clearance_field(const Costmap& cm, uint8_t lethal) {
    const int N = cm.n * cm.n;
    std::vector<uint8_t> obst(std::size_t(N), 0);
    for (int i = 0; i < N; ++i)
        if (cm.cells[std::size_t(i)] >= lethal) obst[std::size_t(i)] = 1;
    const std::vector<float> d2 = edt_squared(obst, cm.n, cm.n);
    std::vector<float> clr(std::size_t(N), 0.0f);
    for (int i = 0; i < N; ++i) clr[std::size_t(i)] = std::sqrt(d2[std::size_t(i)]) * cm.resolution;
    return clr;
}

std::vector<std::pair<float, float>>
medial_axis(const Costmap& cm, const std::vector<float>& clr, float min_clearance_m) {
    std::vector<std::pair<float, float>> pts;
    const int n = cm.n;
    if (int(clr.size()) != n * n) return pts;
    auto at = [&](int ix, int iy) { return clr[std::size_t(iy) * n + ix]; };
    for (int iy = 1; iy < n - 1; ++iy)
        for (int ix = 1; ix < n - 1; ++ix) {
            const float c = at(ix, iy);
            if (c < min_clearance_m) continue;
            // Ridge: a local maximum of the clearance field along at least one axis (the
            // corridor is widest here across that direction = its centreline).
            const bool ridge_x = c >= at(ix-1, iy) && c >= at(ix+1, iy);
            const bool ridge_y = c >= at(ix, iy-1) && c >= at(ix, iy+1);
            if (ridge_x || ridge_y)
                pts.emplace_back(cm.origin_x + (ix + 0.5f) * cm.resolution,
                                 cm.origin_y + (iy + 0.5f) * cm.resolution);
        }
    return pts;
}

} // namespace kist
