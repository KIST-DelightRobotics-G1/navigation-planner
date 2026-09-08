#include "occupancy_grid/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>

namespace kist {

void quat_to_roll_pitch(float qx, float qy, float qz, float qw,
                        float& roll, float& pitch) {
    // ZYX (yaw-pitch-roll) extraction; yaw is discarded.
    const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    float sinp = 2.0f * (qw * qy - qz * qx);
    sinp = std::clamp(sinp, -1.0f, 1.0f);
    pitch = std::asin(sinp);
}

Leveling make_leveling(float roll, float pitch) {
    // Remove the tilt: R = Rx(-roll) * Ry(-pitch).
    const float cr = std::cos(-roll),  sr = std::sin(-roll);
    const float cp = std::cos(-pitch), sp = std::sin(-pitch);
    Leveling L;
    L.r[0] = cp;      L.r[1] = 0.f;  L.r[2] = sp;
    L.r[3] = sr * sp; L.r[4] = cr;   L.r[5] = -sr * cp;
    L.r[6] = -cr * sp;L.r[7] = sr;   L.r[8] = cr * cp;
    return L;
}

Leveling make_leveling_from_normal(float nx, float ny, float nz) {
    // Shortest-arc rotation taking the up-normal n onto +z (Rodrigues):
    //   R = I + [v]x + [v]x^2 / (1 + c),  v = n x z = (ny, -nx, 0),  c = n.z = nz
    float m = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (m < 1e-6f) return {};
    nx /= m; ny /= m; nz /= m;
    if (nz < 0.f) { nx = -nx; ny = -ny; nz = -nz; }   // ensure it points up
    if (nz > 0.9999f) return {};                       // already level
    const float a[9] = { 0.f, 0.f, -nx,
                         0.f, 0.f, -ny,
                          nx,  ny, 0.f };               // [v]x with v=(ny,-nx,0)
    float a2[9];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            float s = 0.f;
            for (int t = 0; t < 3; ++t) s += a[r*3+t] * a[t*3+c];
            a2[r*3+c] = s;
        }
    const float k = 1.f / (1.f + nz);
    Leveling L;
    for (int i = 0; i < 9; ++i)
        L.r[i] = ((i % 4 == 0) ? 1.f : 0.f) + a[i] + k * a2[i];
    return L;
}

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
inline void add_miss(OccupancyGrid& g, int idx, float l_miss, const GridConfig& cfg) {
    g.log_odds[idx] = std::clamp(g.log_odds[idx] - l_miss, cfg.l_min, cfg.l_max);
}
// Free-space carving: Bresenham from the sensor cell (x0,y0) to the hit cell
// (x1,y1), applying a miss to every cell BEFORE the endpoint. Without this the
// grid only ever accumulates hits, so pose jitter smears an obstacle wider and
// wider (a goal in front of the fridge ends up inside it). The endpoint keeps
// its hit (added by the caller after).
inline void ray_clear(OccupancyGrid& g, int x0, int y0, int x1, int y1,
                      float l_miss, const GridConfig& cfg) {
    int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, err = dx + dy;
    for (;;) {
        if (x0 == x1 && y0 == y1) break;              // reached the hit -> leave it occupied
        if (x0 >= 0 && x0 < g.n && y0 >= 0 && y0 < g.n)
            add_miss(g, g.index(x0, y0), l_miss, cfg);
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
}  // namespace

void grid_integrate_lidar(OccupancyGrid& g, const UnitreePointCloud& cloud,
                          const Pose2D& pose, const Leveling& lv, const GridConfig& cfg) {
    const float c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    const float* p = cloud.xyz.data();
    const size_t n = cloud.point_count();
    int rix, riy;
    const bool clear = cfg.l_miss_lidar > 0.0f && g.world_to_cell(pose.x, pose.y, rix, riy);
    for (size_t i = 0; i < n; ++i) {
        float rx, ry, rz;
        lv.apply(p[3*i], p[3*i+1], p[3*i+2], rx, ry, rz);   // level the mount tilt first
        if (rz < cfg.min_z || rz > cfg.max_z) continue;
        const float wx = pose.x + c*rx - s*ry;   // robot -> world
        const float wy = pose.y + s*rx + c*ry;
        int ix, iy;
        if (!g.world_to_cell(wx, wy, ix, iy)) continue;
        if (clear) ray_clear(g, rix, riy, ix, iy, cfg.l_miss_lidar, cfg);  // carve free space
        add_hit(g, g.index(ix, iy), cfg.l_hit_lidar, cfg);
    }
}

void grid_integrate_labeled(OccupancyGrid& g, const LabeledCloud& cloud,
                            const Pose2D& pose, const Leveling& lv, const GridConfig& cfg) {
    const float c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    const size_t n = cloud.size();
    for (size_t i = 0; i < n; ++i) {
        float rx, ry, rz;
        lv.apply(cloud.xyz[3*i], cloud.xyz[3*i+1], cloud.xyz[3*i+2], rx, ry, rz);   // level first
        if (rz < cfg.min_z || rz > cfg.max_z) continue;
        const float wx = pose.x + c*rx - s*ry;   // robot -> world
        const float wy = pose.y + s*rx + c*ry;
        int ix, iy;
        if (!g.world_to_cell(wx, wy, ix, iy)) continue;
        const int idx = g.index(ix, iy);

        // Occupancy: range-weighted trust (far depth is noisy -> weaker hit).
        // Skipped when camera_occupancy is off — then the camera never creates
        // obstacles, it only labels below (LiDAR alone owns occupancy).
        if (cfg.camera_occupancy) {
            const float d = std::sqrt(rx*rx + ry*ry + rz*rz);
            const float t = std::clamp(d / cfg.cam_d_max, 0.0f, 1.0f);
            const float l_hit = cfg.l_hit_cam_near + (cfg.l_hit_cam_far - cfg.l_hit_cam_near) * t;
            add_hit(g, idx, l_hit, cfg);
        }

        // Class: Boyer-Moore majority vote for the dominant class in the cell.
        const uint8_t cl = cloud.label[i];
        if (cl == kNoClass) continue;
        if (g.label_vote[idx] <= 0.0f)       { g.label[idx] = cl; g.label_vote[idx] = 1.0f; }
        else if (g.label[idx] == cl)         { g.label_vote[idx] += 1.0f; }
        else                                 { g.label_vote[idx] -= 1.0f; }
    }
}

} // namespace kist
