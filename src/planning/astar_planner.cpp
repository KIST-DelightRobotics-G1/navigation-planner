#include "planning/astar_planner.hpp"

#include "mapping/edt.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace kist {

namespace {

using P2 = std::pair<float, float>;

inline float seg_len(P2 a, P2 b) { return std::hypot(b.first - a.first, b.second - a.second); }

// Clearance (m, distance to the nearest impassable cell) at an odom point; off-grid = 0.
inline float clr_at(const Costmap& cm, const std::vector<float>& clr, float x, float y) {
    int ix, iy;
    if (!cm.world_to_cell(x, y, ix, iy)) return 0.0f;
    return clr[cm.index(ix, iy)];
}

// min clearance along the segment a->b (sampled every ~step). This is the segmentSafe()
// test: a straight a->b is drivable iff this >= d_safe.
float seg_min_clr(const Costmap& cm, const std::vector<float>& clr, P2 a, P2 b, float step) {
    const int K = std::max(1, int(seg_len(a, b) / step));
    float m = std::numeric_limits<float>::max();
    for (int k = 0; k <= K; ++k) {
        const float t = float(k) / K;
        m = std::min(m, clr_at(cm, clr, a.first + t*(b.first-a.first), a.second + t*(b.second-a.second)));
    }
    return m;
}

// Line-of-sight anchors: from each anchor, jump to the FARTHEST later waypoint still
// reachable by a clear straight line (min clearance >= d_safe) -> the long straight runs.
std::vector<P2> los_anchors(const Costmap& cm, const std::vector<float>& clr,
                            const std::vector<P2>& wps, float d_safe, float step) {
    if (wps.size() <= 2) return wps;
    std::vector<P2> out{wps.front()};
    std::size_t a = 0;
    while (a + 1 < wps.size()) {
        std::size_t b = a + 1;
        for (std::size_t j = wps.size() - 1; j > a + 1; --j)
            if (seg_min_clr(cm, clr, wps[a], wps[j], step) >= d_safe) { b = j; break; }
        out.push_back(wps[b]);
        a = b;
    }
    return out;
}

// Fit the largest circular arc at corner b (between a and c) tangent to both segments and
// clear by d_safe. Fills `arc` (T1..T2 samples) and returns the arc centre+radius. Returns
// false if the corner is near-straight or no feasible radius exists.
bool fit_arc(const Costmap& cm, const std::vector<float>& clr, P2 a, P2 b, P2 c,
             const AStarConfig& cfg, std::vector<P2>& arc, P2& centre, float& radius) {
    arc.clear();
    float v1x = b.first - a.first, v1y = b.second - a.second;
    float v2x = c.first - b.first, v2y = c.second - b.second;
    const float l1 = std::hypot(v1x, v1y), l2 = std::hypot(v2x, v2y);
    if (l1 < 1e-4f || l2 < 1e-4f) return false;
    v1x /= l1; v1y /= l1; v2x /= l2; v2y /= l2;

    const float cross = v1x*v2y - v1y*v2x;                       // turn sign (+ left, - right)
    const float dot   = std::clamp(v1x*v2x + v1y*v2y, -1.0f, 1.0f);
    const float phi   = std::acos(dot);                          // |turn angle|
    if (phi < cfg.corner_angle_min_deg * float(M_PI) / 180.0f) return false;   // near-straight
    const float turn     = (cross >= 0.0f) ? 1.0f : -1.0f;
    const float tan_half = std::tan(phi * 0.5f);
    const float max_dt   = 0.5f * std::min(l1, l2);              // tangent points stay in segments
    const float step     = std::max(0.02f, cm.resolution);

    // Descending R -> the first feasible is the maximum feasible radius (R* = max clearance
    // arc = minimum curvature turn). Fixed step (feasibility need not be monotone in R).
    for (float R = cfg.r_max_m; R >= cfg.r_min_m - 1e-6f; R -= cfg.r_step_m) {
        const float dt = R * tan_half;                          // corner -> tangent point
        if (dt > max_dt) continue;                              // arc too big for the segments
        const P2 T1{b.first - dt*v1x, b.second - dt*v1y};
        const P2 T2{b.first + dt*v2x, b.second + dt*v2y};
        const float n1x = turn * (-v1y), n1y = turn * (v1x);    // perpendicular toward the turn
        const P2 O{T1.first + R*n1x, T1.second + R*n1y};        // arc centre
        float a1 = std::atan2(T1.second - O.second, T1.first - O.first);
        float a2 = std::atan2(T2.second - O.second, T2.first - O.first);
        float sweep = a2 - a1;
        while (sweep >  float(M_PI)) sweep -= 2.0f * float(M_PI);
        while (sweep <= -float(M_PI)) sweep += 2.0f * float(M_PI);

        const int K = std::max(2, int(std::fabs(sweep) * R / step));
        std::vector<P2> samp; samp.reserve(K + 1);
        bool ok = true;
        for (int k = 0; k <= K; ++k) {
            const float ang = a1 + sweep * float(k) / K;
            const P2 pt{O.first + R*std::cos(ang), O.second + R*std::sin(ang)};
            if (clr_at(cm, clr, pt.first, pt.second) < cfg.d_safe_m) { ok = false; break; }
            samp.push_back(pt);
        }
        if (ok) { arc.swap(samp); centre = O; radius = R; return true; }
    }
    return false;
}

// Replace each corner with its largest feasible arc (straight/failed corners kept sharp).
std::vector<P2> arc_smooth(const Costmap& cm, const std::vector<float>& clr,
                           const std::vector<P2>& anchors, const AStarConfig& cfg,
                           std::vector<std::array<float, 3>>& circles) {
    if (anchors.size() < 3) return anchors;
    std::vector<P2> out{anchors.front()};
    for (std::size_t k = 1; k + 1 < anchors.size(); ++k) {
        std::vector<P2> arc; P2 O; float R;
        if (fit_arc(cm, clr, anchors[k-1], anchors[k], anchors[k+1], cfg, arc, O, R)) {
            out.insert(out.end(), arc.begin(), arc.end());
            circles.push_back({O.first, O.second, R});
        } else {
            out.push_back(anchors[k]);
        }
    }
    out.push_back(anchors.back());
    return out;
}

}  // namespace

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

    const float res = cm.resolution;
    const bool  ign = cfg_.goal_ignore_radius_m > 0.0f;
    const float ign_c2 = ign ? (cfg_.goal_ignore_radius_m / res) * (cfg_.goal_ignore_radius_m / res) : 0.0f;
    auto near_goal = [&](int ix, int iy) {
        const float dx = float(ix - gix), dy = float(iy - giy);
        return ign && (dx*dx + dy*dy) <= ign_c2;
    };
    if (cm.cells[gi] >= obs && !near_goal(gix, giy)) return path;

    // Clearance field (m, distance to the nearest lethal cell). Used BOTH as the A* centre
    // preference (route through the middle) and by the smoother (LOS + arc feasibility).
    std::vector<uint8_t> obst(std::size_t(N), 0);
    for (int i = 0; i < N; ++i) if (cm.cells[std::size_t(i)] >= obs) obst[std::size_t(i)] = 1;
    const std::vector<float> d2 = edt_squared(obst, n, n);
    std::vector<float> clr(std::size_t(N), 0.0f);
    for (int i = 0; i < N; ++i) clr[std::size_t(i)] = std::sqrt(d2[std::size_t(i)]) * res;

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
    path.raw_waypoints = path.waypoints;   // keep the raw A* grid path for comparison

    // Smooth: straighten (LOS, EDT-based) then fit the largest feasible arc at each corner.
    if (cfg_.smooth && path.waypoints.size() > 2) {
        const float step = std::max(0.02f, res);
        const std::vector<P2> anchors = los_anchors(cm, clr, path.waypoints, cfg_.d_safe_m, step);
        path.waypoints = arc_smooth(cm, clr, anchors, cfg_, path.turn_circles);
    }
    return path;
}

} // namespace kist
