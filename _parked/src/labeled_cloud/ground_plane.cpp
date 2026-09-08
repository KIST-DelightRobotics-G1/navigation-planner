#include "labeled_cloud/ground_plane.hpp"

#include <cmath>
#include <cstdint>
#include <random>

namespace kist {

namespace {
inline float dist_to_plane(const GroundPlane& p, float x, float y, float z) {
    return std::fabs(p.nx * x + p.ny * y + p.nz * z + p.d);
}
}  // namespace

GroundPlane fit_ground_plane(const std::vector<float>& xyz,
                             const GroundPlaneConfig& cfg,
                             const GroundPlane& prev) {
    if (!cfg.enabled || xyz.size() < 9) return prev;

    // Ground candidates: low points only (near the floor even when tilted, since
    // the mount pivot sits roughly over the floor origin). Cuts the walls out.
    const size_t n = xyz.size() / 3;
    std::vector<uint32_t> cand;
    cand.reserve(n / 2);
    for (size_t i = 0; i < n; ++i)
        if (xyz[3*i+2] < cfg.candidate_z_max) cand.push_back(uint32_t(i));
    if (cand.size() < size_t(cfg.min_inliers)) return prev;

    const float cos_max = std::cos(cfg.max_tilt_deg * 3.14159265f / 180.f);

    std::mt19937 rng(uint32_t(cand.size()));   // deterministic per frame
    std::uniform_int_distribution<size_t> pick(0, cand.size() - 1);

    GroundPlane best;
    for (int it = 0; it < cfg.iterations; ++it) {
        const uint32_t a = cand[pick(rng)], b = cand[pick(rng)], c = cand[pick(rng)];
        if (a == b || b == c || a == c) continue;
        const float ax = xyz[3*a], ay = xyz[3*a+1], az = xyz[3*a+2];
        const float bx = xyz[3*b], by = xyz[3*b+1], bz = xyz[3*b+2];
        const float cx = xyz[3*c], cy = xyz[3*c+1], cz = xyz[3*c+2];
        // normal = (b-a) x (c-a)
        float nx = (by-ay)*(cz-az) - (bz-az)*(cy-ay);
        float ny = (bz-az)*(cx-ax) - (bx-ax)*(cz-az);
        float nz = (bx-ax)*(cy-ay) - (by-ay)*(cx-ax);
        const float m = std::sqrt(nx*nx + ny*ny + nz*nz);
        if (m < 1e-9f) continue;
        nx /= m; ny /= m; nz /= m;
        if (nz < 0) { nx = -nx; ny = -ny; nz = -nz; }   // point up
        if (nz < cos_max) continue;                      // reject non-horizontal (walls)

        GroundPlane cur{nx, ny, nz, -(nx*ax + ny*ay + nz*az), 0, false};
        int inl = 0;
        for (uint32_t idx : cand)
            if (dist_to_plane(cur, xyz[3*idx], xyz[3*idx+1], xyz[3*idx+2]) < cfg.inlier_eps_m)
                ++inl;
        if (inl > best.inliers) { cur.inliers = inl; best = cur; }
    }

    if (best.inliers < cfg.min_inliers) return prev;   // no confident floor -> keep prior

    best.valid = true;
    // Temporal smoothing: blend the normal with the previous (reduces gait jitter).
    if (prev.valid && cfg.smooth < 1.f) {
        const float a = cfg.smooth;
        float nx = a*best.nx + (1-a)*prev.nx;
        float ny = a*best.ny + (1-a)*prev.ny;
        float nz = a*best.nz + (1-a)*prev.nz;
        const float m = std::sqrt(nx*nx + ny*ny + nz*nz);
        if (m > 1e-6f) { best.nx = nx/m; best.ny = ny/m; best.nz = nz/m; }
    }
    return best;
}

} // namespace kist
