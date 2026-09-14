#include "route_planner/planner/astar_planner/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace kist {

Path AStarPlanner::plan(const Costmap& cm, const std::vector<float>& clr,
                        std::pair<float, float> start, std::pair<float, float> goal) const {
    Path path;
    path.stamp_ns = cm.stamp_ns;
    if (cm.empty() || int(clr.size()) != cm.n * cm.n) return path;

    const int n = cm.n, N = n * n;
    int six, siy, gix, giy;
    if (!cm.world_to_cell(start.first, start.second, six, siy)) return path;
    if (!cm.world_to_cell(goal.first,  goal.second,  gix, giy)) return path;

    const int     si  = cm.index(six, siy);
    const int     gi  = cm.index(gix, giy);
    const uint8_t obs = uint8_t(cfg_.obs_cost);
    if (si == gi) return path;

    const float res = cm.resolution;
    const bool  ign = cfg_.goal_ignore_radius_m > 0.0f;
    const float ign_c2 = ign ? (cfg_.goal_ignore_radius_m / res) * (cfg_.goal_ignore_radius_m / res) : 0.0f;
    auto near_goal = [&](int ix, int iy) {
        const float dx = float(ix - gix), dy = float(iy - giy);
        return ign && (dx*dx + dy*dy) <= ign_c2;
    };
    if (cm.cells[gi] >= obs && !near_goal(gix, giy)) return path;

    constexpr float kInf = std::numeric_limits<float>::infinity();
    std::vector<float> g(N, kInf);
    std::vector<int>   parent(N, -1);
    std::vector<char>  closed(N, 0);

    auto heuristic = [&](int ix, int iy) {
        return cfg_.heuristic_weight * cfg_.base_cost *
               float(std::max(std::abs(gix - ix), std::abs(giy - iy)));
    };

    using Entry = std::pair<float, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    g[si] = 0.0f;
    open.push({heuristic(six, siy), si});

    static constexpr int   dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static constexpr int   dy[] = {-1,-1,-1,  0, 0,  1, 1, 1};
    static constexpr float dm[] = {1.41421356f,1,1.41421356f, 1,1, 1.41421356f,1,1.41421356f};

    int  iters = 0;
    bool found = false;
    while (!open.empty() && iters < cfg_.max_iterations) {
        const int idx = open.top().second;
        open.pop();
        if (closed[idx]) continue;
        closed[idx] = 1;
        ++iters;
        if (idx == gi) { found = true; break; }

        const int ix = idx % n, iy = idx / n;
        for (int d = 0; d < 8; ++d) {
            const int nx = ix + dx[d], ny = iy + dy[d];
            if (nx < 0 || nx >= n || ny < 0 || ny >= n) continue;
            const int nidx = cm.index(nx, ny);
            if (closed[nidx]) continue;
            const bool ng_near = near_goal(nx, ny);
            if (!ng_near && cm.cells[nidx] >= obs) continue;      // lethal (ignored near goal)
            // Centre preference: penalise cells with clearance below ref_clr_m; 0 beyond.
            const float pen = ng_near ? 0.0f
                              : cfg_.w_center * std::max(0.0f, cfg_.ref_clr_m - clr[nidx]);
            const float ng  = g[idx] + (cfg_.base_cost + pen) * dm[d];
            if (ng < g[nidx]) {
                g[nidx] = ng;
                parent[nidx] = idx;
                open.push({ng + heuristic(nx, ny), nidx});
            }
        }
    }
    if (!found) return path;

    for (int idx = gi; idx != -1; idx = parent[idx]) {
        const int ix = idx % n, iy = idx / n;
        path.waypoints.emplace_back(cm.origin_x + (ix + 0.5f) * res,
                                    cm.origin_y + (iy + 0.5f) * res);
    }
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    path.raw_waypoints = path.waypoints;   // raw grid route (smoothing is RouteSmoother's job)
    return path;
}

} // namespace kist
