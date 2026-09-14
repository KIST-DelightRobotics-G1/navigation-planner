#include "route_planner/planner/route_smoothing/route_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kist {

namespace {

using P2 = std::pair<float, float>;

inline float seg_len(P2 a, P2 b) { return std::hypot(b.first - a.first, b.second - a.second); }

// Clearance (m) at an odom point; off-grid = 0.
inline float clr_at(const Costmap& cm, const std::vector<float>& clr, float x, float y) {
    int ix, iy;
    if (!cm.world_to_cell(x, y, ix, iy)) return 0.0f;
    return clr[cm.index(ix, iy)];
}

// min clearance along the segment a->b (sampled) — the segmentSafe() test.
float seg_min_clr(const Costmap& cm, const std::vector<float>& clr, P2 a, P2 b, float step) {
    const int K = std::max(1, int(seg_len(a, b) / step));
    float m = std::numeric_limits<float>::max();
    for (int k = 0; k <= K; ++k) {
        const float t = float(k) / K;
        m = std::min(m, clr_at(cm, clr, a.first + t*(b.first-a.first), a.second + t*(b.second-a.second)));
    }
    return m;
}

// LOS anchors: from each anchor, jump to the FARTHEST later waypoint still reachable by a
// clear straight line (min clearance >= d_safe) -> the long straight runs.
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
// clear by d_safe. Fills `arc` (T1..T2 samples) + the arc centre/radius. False if the corner
// is near-straight or no feasible radius exists.
bool fit_arc(const Costmap& cm, const std::vector<float>& clr, P2 a, P2 b, P2 c,
             const RouteSmoothConfig& cfg, std::vector<P2>& arc, P2& centre, float& radius) {
    arc.clear();
    float v1x = b.first - a.first, v1y = b.second - a.second;
    float v2x = c.first - b.first, v2y = c.second - b.second;
    const float l1 = std::hypot(v1x, v1y), l2 = std::hypot(v2x, v2y);
    if (l1 < 1e-4f || l2 < 1e-4f) return false;
    v1x /= l1; v1y /= l1; v2x /= l2; v2y /= l2;

    const float cross = v1x*v2y - v1y*v2x;
    const float dot   = std::clamp(v1x*v2x + v1y*v2y, -1.0f, 1.0f);
    const float phi   = std::acos(dot);
    if (phi < cfg.corner_angle_min_deg * float(M_PI) / 180.0f) return false;   // near-straight
    const float turn     = (cross >= 0.0f) ? 1.0f : -1.0f;
    const float tan_half = std::tan(phi * 0.5f);
    const float max_dt   = 0.5f * std::min(l1, l2);
    const float step     = std::max(0.02f, cm.resolution);

    for (float R = cfg.r_max_m; R >= cfg.r_min_m - 1e-6f; R -= cfg.r_step_m) {
        const float dt = R * tan_half;
        if (dt > max_dt) continue;
        const P2 T1{b.first - dt*v1x, b.second - dt*v1y};
        const P2 T2{b.first + dt*v2x, b.second + dt*v2y};
        const float n1x = turn * (-v1y), n1y = turn * (v1x);
        const P2 O{T1.first + R*n1x, T1.second + R*n1y};
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

}  // namespace

void RouteSmoother::smooth(const Costmap& cm, const std::vector<float>& clr,
                           const std::vector<P2>& raw,
                           std::vector<P2>& out_waypoints,
                           std::vector<std::array<float, 3>>& out_circles) const {
    out_waypoints.clear();
    out_circles.clear();
    if (raw.size() <= 2) { out_waypoints = raw; return; }

    const float step = std::max(0.02f, cm.resolution);
    const std::vector<P2> anchors = los_anchors(cm, clr, raw, cfg_.d_safe_m, step);
    if (anchors.size() < 3) { out_waypoints = anchors; return; }

    out_waypoints.push_back(anchors.front());
    for (std::size_t k = 1; k + 1 < anchors.size(); ++k) {
        std::vector<P2> arc; P2 O; float R;
        if (fit_arc(cm, clr, anchors[k-1], anchors[k], anchors[k+1], cfg_, arc, O, R)) {
            out_waypoints.insert(out_waypoints.end(), arc.begin(), arc.end());
            out_circles.push_back({O.first, O.second, R});
        } else {
            out_waypoints.push_back(anchors[k]);
        }
    }
    out_waypoints.push_back(anchors.back());
}

} // namespace kist
