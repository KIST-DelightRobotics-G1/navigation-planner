#pragma once

// Replaces FAST-LIO's preprocess.cpp (which parsed Livox CustomMsg): converts our
// DDS Mid-360 frame straight into the FAST-LIO PointCloudXYZI, since UnitreePointCloud
// already carries per-point time. The KEY mapping is per-point time (ns offset from
// the frame stamp) -> `curvature` in MILLISECONDS, which is what FAST-LIO's
// undistortion expects (it reads curvature/1000 as a seconds offset).
//
// Drops points within `blind` metres (near-field self/noise returns) and keeps every
// `filter_num`-th point (downsample). Needs the time channel; intensity optional.

#include "unitree/unitree_pointcloud.hpp"
#include "lio/common_lib.h"          // PointType, PointCloudXYZI

#include <algorithm>
#include <cstddef>

namespace kist {

inline void to_fastlio_cloud(const UnitreePointCloud& in, PointCloudXYZI& out,
                             double blind = 0.5, int filter_num = 1) {
    const std::size_t n     = in.point_count();
    const bool        has_i = in.has_intensity();
    const bool        has_t = in.has_time();
    const int         step  = std::max(1, filter_num);
    const double      blind2 = blind * blind;

    out.clear();
    out.reserve(n / step + 1);
    for (std::size_t i = 0; i < n; i += step) {
        const float x = in.xyz[3*i], y = in.xyz[3*i+1], z = in.xyz[3*i+2];
        if (double(x)*x + double(y)*y + double(z)*z < blind2) continue;   // blind radius

        PointType p;
        p.x = x; p.y = y; p.z = z;
        p.intensity = has_i ? in.intensity[i] : 0.0f;
        p.curvature = has_t ? in.time[i] * 1e-6f : 0.0f;                  // ns -> ms
        out.push_back(p);
    }
}

} // namespace kist
