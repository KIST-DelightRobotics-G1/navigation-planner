#include "route_planner/obstacle_grid_builder/obstacle_voxel_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace kist {

void voxel_reset(ObstacleVoxelGrid& v, const ObstacleGridConfig& cfg,
                 float center_x, float center_y) {
    v.resolution   = cfg.resolution_m;
    v.resolution_z = cfg.resolution_z_m;
    v.n = std::max(1, int(std::lround(2.0f * cfg.half_extent_m / cfg.resolution_m)));
    v.k = std::max(1, int(std::lround((cfg.max_height_m - cfg.min_height_m) / cfg.resolution_z_m)));
    v.origin_x   = center_x - cfg.half_extent_m;
    v.origin_y   = center_y - cfg.half_extent_m;
    v.min_height = cfg.min_height_m;
    v.log_odds.assign(std::size_t(v.n) * v.n * v.k, 0.0f);   // 0 -> P 0.5 (unknown)
}

void voxel_decay(ObstacleVoxelGrid& v, const ObstacleGridConfig& cfg) {
    for (float& l : v.log_odds) l *= cfg.decay;
}

void voxel_recenter(ObstacleVoxelGrid& v, float center_x, float center_y) {
    if (v.empty()) return;
    const float half = 0.5f * v.n * v.resolution;               // half_extent
    const float want_ox = center_x - half, want_oy = center_y - half;
    const int sx = int(std::lround((want_ox - v.origin_x) / v.resolution));
    const int sy = int(std::lround((want_oy - v.origin_y) / v.resolution));
    if (sx == 0 && sy == 0) return;                             // sub-cell move -> nothing to do

    // New column (ix,iy) inherits old column (ix+sx, iy+sy); off-grid -> cleared. A whole
    // z-column (k voxels) moves as a unit, so only the xy anchor changes.
    std::vector<float> nb(v.log_odds.size(), 0.0f);
    const int stride = v.k;
    for (int iy = 0; iy < v.n; ++iy) {
        const int oy = iy + sy;
        if (oy < 0 || oy >= v.n) continue;
        for (int ix = 0; ix < v.n; ++ix) {
            const int ox = ix + sx;
            if (ox < 0 || ox >= v.n) continue;
            std::memcpy(&nb[(iy * v.n + ix) * stride],
                        &v.log_odds[(oy * v.n + ox) * stride],
                        stride * sizeof(float));
        }
    }
    v.log_odds.swap(nb);
    v.origin_x += sx * v.resolution;                            // cell-aligned move
    v.origin_y += sy * v.resolution;
}

namespace {
inline void add_hit(ObstacleVoxelGrid& v, int idx, const ObstacleGridConfig& cfg) {
    v.log_odds[idx] = std::clamp(v.log_odds[idx] + cfg.l_hit, cfg.l_min, cfg.l_max);
}
inline void add_miss(ObstacleVoxelGrid& v, int ix, int iy, int iz, const ObstacleGridConfig& cfg) {
    if (ix < 0 || ix >= v.n || iy < 0 || iy >= v.n || iz < 0 || iz >= v.k) return;   // outside grid
    const int idx = v.index(ix, iy, iz);
    v.log_odds[idx] = std::clamp(v.log_odds[idx] - cfg.l_miss, cfg.l_min, cfg.l_max);
}

// 3D Bresenham (integer voxel traversal) from (x0,y0,z0) to (x1,y1,z1), applying a miss to
// every voxel BEFORE the endpoint (the endpoint keeps its hit, added by the caller). The
// start may be outside the grid; add_miss bounds-checks each voxel. Driven by the dominant
// axis with two error terms — the standard 3D line algorithm.
void ray_clear_3d(ObstacleVoxelGrid& v, int x0, int y0, int z0,
                  int x1, int y1, int z1, const ObstacleGridConfig& cfg) {
    const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0), dz = std::abs(z1 - z0);
    const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, sz = z0 < z1 ? 1 : -1;

    if (dx >= dy && dx >= dz) {                 // x dominant
        int ey = 2*dy - dx, ez = 2*dz - dx;
        while (x0 != x1) {
            add_miss(v, x0, y0, z0, cfg);
            if (ey > 0) { y0 += sy; ey -= 2*dx; }
            if (ez > 0) { z0 += sz; ez -= 2*dx; }
            ey += 2*dy; ez += 2*dz; x0 += sx;
        }
    } else if (dy >= dx && dy >= dz) {          // y dominant
        int ex = 2*dx - dy, ez = 2*dz - dy;
        while (y0 != y1) {
            add_miss(v, x0, y0, z0, cfg);
            if (ex > 0) { x0 += sx; ex -= 2*dy; }
            if (ez > 0) { z0 += sz; ez -= 2*dy; }
            ex += 2*dx; ez += 2*dz; y0 += sy;
        }
    } else {                                    // z dominant
        int ex = 2*dx - dz, ey = 2*dy - dz;
        while (z0 != z1) {
            add_miss(v, x0, y0, z0, cfg);
            if (ex > 0) { x0 += sx; ex -= 2*dz; }
            if (ey > 0) { y0 += sy; ey -= 2*dz; }
            ex += 2*dx; ey += 2*dy; z0 += sz;
        }
    }
}
}  // namespace

void voxel_integrate(ObstacleVoxelGrid& v, const LioCloud& scan,
                     float ray_ox, float ray_oy, float ray_oz, float floor_z_odom,
                     const ObstacleGridConfig& cfg) {
    if (v.empty()) return;

    // Sensor voxel (ray origin). It may sit outside the grid (e.g. above max_height); the
    // 3D raycast clips per-voxel, so an out-of-range start is fine.
    int six, siy;
    v.col_to_cell(ray_ox, ray_oy, six, siy);           // may be outside; used as line start
    const float sensor_h = ray_oz - floor_z_odom;      // lidar height above floor
    const int siz = v.height_to_bin(sensor_h);
    const bool carve = cfg.l_miss > 0.0f;

    // Dynamic band top: just below the lidar (= robot head). Points above it are overhead /
    // ceiling — dropped. Band = [min_height_m, sensor_h - upper_margin_below_lidar_m].
    const float top_h = sensor_h - cfg.upper_margin_below_lidar_m;

    const float max_r2  = cfg.max_range_m * cfg.max_range_m;
    const float self_r2 = cfg.self_radius_m * cfg.self_radius_m;
    const float* p = scan.xyz.data();
    const std::size_t n = scan.point_count();
    for (std::size_t i = 0; i < n; ++i) {
        const float px = p[3*i], py = p[3*i+1], pz = p[3*i+2];

        const float dx = px - ray_ox, dy = py - ray_oy;
        const float d2 = dx*dx + dy*dy;
        if (d2 < self_r2 || d2 > max_r2) continue;     // drop self-body (near) + far noise

        // Range-dependent floor cut: lift the bottom with range so far scattered floor
        // returns fall below the band (near obstacles keep the low cut).
        const float min_h = cfg.min_height_m + cfg.floor_cut_slope_m_per_m * std::sqrt(d2);
        const float h = pz - floor_z_odom;             // height above floor
        if (h < min_h || h > top_h) continue;          // band [floor+min(range), lidar-margin]
        const int iz = v.height_to_bin(h);
        if (iz < 0 || iz >= v.k) continue;             // grid-bounds safety

        int ix, iy;
        if (!v.col_to_cell(px, py, ix, iy)) continue;  // endpoint outside the window

        if (carve) ray_clear_3d(v, six, siy, siz, ix, iy, iz, cfg);
        add_hit(v, v.index(ix, iy, iz), cfg);
    }
    v.stamp_ns = scan.stamp_ns;
}

void voxel_project_to_2d(const ObstacleVoxelGrid& v, ObstacleGrid& out) {
    out.n          = v.n;
    out.resolution = v.resolution;
    out.origin_x   = v.origin_x;
    out.origin_y   = v.origin_y;
    out.stamp_ns   = v.stamp_ns;
    out.log_odds.assign(std::size_t(v.n) * v.n, 0.0f);
    if (v.empty()) return;

    // Projection rule: OCCUPIED wins, else FREE. A column is occupied if ANY voxel has
    // positive evidence (an obstacle at any height blocks the cell) -> take the max. If no
    // voxel is occupied, take the MIN (most-cleared) so a column that was swept free at
    // some height reads free, instead of unknown just because a high voxel was never
    // observed. All-zero columns stay unknown (0). This keeps obstacles safe while letting
    // open space read as traversable.
    for (int iy = 0; iy < v.n; ++iy)
        for (int ix = 0; ix < v.n; ++ix) {
            const int base = (iy * v.n + ix) * v.k;
            float mx = v.log_odds[base], mn = v.log_odds[base];
            for (int iz = 1; iz < v.k; ++iz) {
                const float l = v.log_odds[base + iz];
                mx = std::max(mx, l);
                mn = std::min(mn, l);
            }
            out.log_odds[iy * v.n + ix] = (mx > 1e-3f) ? mx : mn;   // occupied else free/unknown
        }
}

} // namespace kist
