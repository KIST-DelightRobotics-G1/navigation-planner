#include "planner/astar/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace kist {

Path AStarPlanner::plan(const Costmap& cm,
                        std::pair<float, float> start,
                        std::pair<float, float> goal) const {
    Path path;
    path.stamp_ns = cm.stamp_ns;
    if (cm.empty()) return path;

    const int n = cm.n, N = n * n;
    int six, siy, gix, giy;
    if (!cm.world_to_cell(start.first, start.second, six, siy)) return path;
    if (!cm.world_to_cell(goal.first,  goal.second,  gix, giy)) return path;

    const int     si  = cm.index(six, siy);
    const int     gi  = cm.index(gix, giy);
    const uint8_t obs = uint8_t(cfg_.obs_cost);
    if (si == gi) return path;
    // Goal-approach: cells within goal_ignore_radius of the goal are treated as
    // free (cost + lethal ignored), so a destination next to an obstacle stays
    // reachable. Precompute the squared radius in cells.
    const float res_ = cm.resolution;
    const bool  ign  = cfg_.goal_ignore_radius_m > 0.0f;
    const float ign_c2 = ign ? (cfg_.goal_ignore_radius_m / res_) * (cfg_.goal_ignore_radius_m / res_) : 0.0f;
    auto near_goal = [&](int ix, int iy) {
        const float dx = float(ix - gix), dy = float(iy - giy);
        return ign && (dx*dx + dy*dy) <= ign_c2;
    };
    // The GOAL must be free — unless the ignore radius covers it (then a lethal
    // goal in an inflation zone is still allowed). The START is never rejected.
    if (cm.cells[gi] >= obs && !near_goal(gix, giy)) return path;

    constexpr float kInf = std::numeric_limits<float>::infinity();
    std::vector<float> g(N, kInf);
    std::vector<float> dist(N, kInf);   // travel distance (cells) along the best path to a cell
    std::vector<int>   parent(N, -1);
    std::vector<char>  closed(N, 0);

    auto heuristic = [&](int ix, int iy) {
        return cfg_.heuristic_weight * cfg_.base_cost *
               float(std::max(std::abs(gix - ix), std::abs(giy - iy)));
    };

    using Entry = std::pair<float, int>;   // (f, cell)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    g[si] = 0.0f;
    dist[si] = 0.0f;
    open.push({heuristic(six, siy), si});

    static constexpr int   dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static constexpr int   dy[] = {-1,-1,-1,  0, 0,  1, 1, 1};
    static constexpr float dm[] = {1.41421356f,1,1.41421356f, 1,1, 1.41421356f,1,1.41421356f};

    // Distance-trust weight for a cell: 1 at the robot, decaying to trust_min at
    // range. Scales only the soft cost, so far obstacles shape the path loosely
    // (we replan as we approach) while near ones are respected in full.
    const float res = cm.resolution;
    auto trust = [&](int ix, int iy) {
        if (cfg_.trust_falloff_m <= 0.0f) return 1.0f;
        const float dm_ = std::sqrt(float((ix - six) * (ix - six) +
                                          (iy - siy) * (iy - siy))) * res;
        return cfg_.trust_min +
               (1.0f - cfg_.trust_min) * std::exp(-dm_ / cfg_.trust_falloff_m);
    };

    // Path-length cap: reject any route whose accumulated travel distance exceeds
    // max_path_len_m (tracked per node in `dist`, cells). Straight-line start->goal
    // is a lower bound on any path, so a goal already farther than the cap can be
    // rejected up front.
    const bool  cap_len   = cfg_.max_path_len_m > 0.0f;
    const float max_cells = cap_len ? cfg_.max_path_len_m / res : 0.0f;
    if (cap_len) {
        const float dx = float(gix - six), dy = float(giy - siy);
        if (std::sqrt(dx * dx + dy * dy) > max_cells) return path;
    }

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
            const uint8_t cost = cm.cells[nidx];
            const bool ng_near = near_goal(nx, ny);           // goal-approach zone
            if (!ng_near && cost >= obs) continue;            // impassable (ignored near the goal)
            const float nd = dist[idx] + dm[d];               // travel distance along this path
            if (cap_len && nd > max_cells) continue;          // would exceed the length cap
            const float eff = ng_near ? 0.0f : float(cost);   // ignore cost near the goal
            const float ng = g[idx] +
                (cfg_.base_cost + eff * trust(nx, ny)) * dm[d];
            if (ng < g[nidx]) {
                g[nidx] = ng;
                dist[nidx] = nd;
                parent[nidx] = idx;
                open.push({ng + heuristic(nx, ny), nidx});
            }
        }
    }
    if (!found) return path;

    for (int idx = gi; idx != -1; idx = parent[idx]) {
        const int ix = idx % n, iy = idx / n;
        path.waypoints.emplace_back(cm.origin_x + (ix + 0.5f) * cm.resolution,
                                    cm.origin_y + (iy + 0.5f) * cm.resolution);
    }
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    return path;
}

} // namespace kist
