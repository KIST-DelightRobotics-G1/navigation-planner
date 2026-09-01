#pragma once

// Deproject an aligned depth frame into a labeled 3D point cloud — each point
// tagged with the semantic class at its pixel. Depth is reprojected into the
// color frame on the transmitter (ext align_to_color), so depth[u,v], the color
// image, and the YOLO mask all share ONE pixel grid + the color intrinsics that
// now ride on every depth frame (fx/fy/cx/cy). One pass over the depth image:
//   Z = raw * depth_scale;  X = (u - cx) * Z / fx;  Y = (v - cy) * Z / fy
//   label = seg.class_at(u, v)
// Points come out in the RealSense optical frame: +X right, +Y down, +Z forward
// (metres). No extrinsics yet — placing the cloud in the robot/planning frame,
// and fusing it with the LiDAR cloud, are later steps.

#include "realsense/depth_frame.hpp"                              // ext (embedded)
#include "cv/yolo/semantic_segmentation/yolo_sem_seg_result.hpp"  // SemSegResult

#include <cstdint>
#include <vector>

namespace kist {

// No-class sentinel for points whose pixel fell outside the semantic map
// (letterbox border) — kept in the cloud, flagged rather than dropped.
constexpr uint8_t kNoClass = 255;

// Structure-of-arrays labeled cloud. xyz is packed [x0,y0,z0, x1,y1,z1, ...];
// label[i] is the class id of point i (kNoClass where the mask had none).
struct LabeledCloud {
    int64_t              stamp_ns = 0;   // carried from the depth frame
    std::vector<float>   xyz;
    std::vector<uint8_t> label;

    size_t size()  const { return label.size(); }   // number of points
    bool   empty() const { return label.empty(); }
    void   clear() { xyz.clear(); label.clear(); }
};

// Deproject `d` into `out`, labeling each point via `seg`. Skips invalid depth
// (raw 0) and, when `require_label` is true, pixels with no class. `stride`
// subsamples both axes (1 = every pixel). Reuses out's storage across calls.
inline void fuse_depth_semantic(const DepthFrame& d, const SemSegResult& seg,
                                LabeledCloud& out, int stride = 1,
                                bool require_label = false) {
    out.clear();
    out.stamp_ns = d.stamp_ns;
    // Without intrinsics the deprojection is undefined — bail rather than emit
    // a garbage cloud (fx/fy are 0 on a pre-intrinsics transmitter).
    if (d.data.empty() || d.fx <= 0.f || d.fy <= 0.f) return;
    if (stride < 1) stride = 1;

    const float inv_fx = 1.f / d.fx, inv_fy = 1.f / d.fy;
    const int   W = d.width, H = d.height;

    const size_t cap = size_t(W / stride + 1) * size_t(H / stride + 1);
    out.xyz.reserve(cap * 3);
    out.label.reserve(cap);

    for (int v = 0; v < H; v += stride) {
        const auto* row = reinterpret_cast<const uint16_t*>(
            d.data.data() + size_t(v) * d.stride_bytes);
        for (int u = 0; u < W; u += stride) {
            const uint16_t raw = row[u];
            if (raw == 0) continue;                       // no return / invalid
            const int cls = seg.class_at(float(u), float(v));
            if (require_label && cls < 0) continue;
            const float Z = raw * d.depth_scale;
            out.xyz.push_back((u - d.cx) * Z * inv_fx);
            out.xyz.push_back((v - d.cy) * Z * inv_fy);
            out.xyz.push_back(Z);
            out.label.push_back(cls < 0 ? kNoClass : uint8_t(cls));
        }
    }
}

} // namespace kist
