#include "occupancy_grid/occupancy_grid.hpp"

#include <algorithm>

namespace kist {

void grid_reset(OccupancyGrid& g, const GridConfig& cfg, float center_x, float center_y) {
    g.resolution = cfg.resolution_m;
    g.n = std::max(1, int(std::lround(2.0f * cfg.half_extent_m / cfg.resolution_m)));
    g.origin_x = center_x - cfg.half_extent_m;
    g.origin_y = center_y - cfg.half_extent_m;
    g.robot_x = center_x;
    g.robot_y = center_y;
    const size_t cells = size_t(g.n) * g.n;
    g.log_odds.assign(cells, 0.0f);       // log-odds 0 -> P(occ) 0.5
    g.label.assign(cells, kNoClass);
    g.label_vote.assign(cells, 0.0f);
}

void grid_decay(OccupancyGrid& g, const GridConfig& cfg) {
    for (size_t i = 0; i < g.log_odds.size(); ++i) {
        const uint8_t c = g.label[i];
        // Dynamic classes (people, ...) fade fast; static / unlabeled slow.
        const float d = (c != kNoClass && cfg.is_dynamic[c]) ? cfg.decay_dynamic
                                                             : cfg.decay_static;
        g.log_odds[i]   *= d;
        g.label_vote[i] *= d;
        if (g.label_vote[i] < 0.05f) g.label[i] = kNoClass;   // vote faded -> unknown
    }
}

namespace {
inline void add_hit(OccupancyGrid& g, int idx, float l_hit, const GridConfig& cfg) {
    g.log_odds[idx] = std::clamp(g.log_odds[idx] + l_hit, cfg.l_min, cfg.l_max);
}
}  // namespace

void grid_integrate_lidar(OccupancyGrid& g, const UnitreePointCloud& cloud,
                          const Pose2D& pose, const GridConfig& cfg) {
    const float c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    const float* p = cloud.xyz.data();
    const size_t n = cloud.point_count();
    for (size_t i = 0; i < n; ++i) {
        const float rx = p[3*i], ry = p[3*i+1], rz = p[3*i+2];
        if (rz < cfg.min_z || rz > cfg.max_z) continue;
        const float wx = pose.x + c*rx - s*ry;   // robot -> world
        const float wy = pose.y + s*rx + c*ry;
        int ix, iy;
        if (!g.world_to_cell(wx, wy, ix, iy)) continue;
        add_hit(g, g.index(ix, iy), cfg.l_hit_lidar, cfg);
    }
}

void grid_integrate_labeled(OccupancyGrid& g, const LabeledCloud& cloud,
                            const Pose2D& pose, const GridConfig& cfg) {
    const float c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    const size_t n = cloud.size();
    for (size_t i = 0; i < n; ++i) {
        const float rx = cloud.xyz[3*i], ry = cloud.xyz[3*i+1], rz = cloud.xyz[3*i+2];
        if (rz < cfg.min_z || rz > cfg.max_z) continue;
        const float wx = pose.x + c*rx - s*ry;   // robot -> world
        const float wy = pose.y + s*rx + c*ry;
        int ix, iy;
        if (!g.world_to_cell(wx, wy, ix, iy)) continue;
        const int idx = g.index(ix, iy);

        // Occupancy: range-weighted trust (far depth is noisy -> weaker hit).
        const float d = std::sqrt(rx*rx + ry*ry + rz*rz);
        const float t = std::clamp(d / cfg.cam_d_max, 0.0f, 1.0f);
        const float l_hit = cfg.l_hit_cam_near + (cfg.l_hit_cam_far - cfg.l_hit_cam_near) * t;
        add_hit(g, idx, l_hit, cfg);

        // Class: Boyer-Moore majority vote for the dominant class in the cell.
        const uint8_t cl = cloud.label[i];
        if (cl == kNoClass) continue;
        if (g.label_vote[idx] <= 0.0f)       { g.label[idx] = cl; g.label_vote[idx] = 1.0f; }
        else if (g.label[idx] == cl)         { g.label_vote[idx] += 1.0f; }
        else                                 { g.label_vote[idx] -= 1.0f; }
    }
}

} // namespace kist
