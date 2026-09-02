#pragma once

// LabeledCloud — a point cloud whose points each carry a semantic class id, plus
// the function that builds one from a depth frame + semantic mask. Depth is
// reprojected into the color frame on the transmitter (ext align_to_color), so
// depth[u,v], the color image, and the YOLO mask share one pixel grid + the
// color intrinsics that ride on every depth frame (fx/fy/cx/cy). One pass over
// the depth image:
//   Z = raw * depth_scale;  X = (u - cx) * Z / fx;  Y = (v - cy) * Z / fy
//   label = seg.class_at(u, v)
// Points come out in the RealSense optical frame: +X right, +Y down, +Z forward
// (metres). transform_cloud() (camera_extrinsics.hpp) lifts them to the robot
// frame; LabeledCloudGenerator runs both on a worker thread.

#include "realsense/depth_frame.hpp"                              // ext (embedded)
#include "cv/yolo/semantic_segmentation/yolo_sem_seg_frame.hpp"  // SemSegFrame

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
// Output is in the camera optical frame.
void fuse_depth_semantic(const DepthFrame& d, const SemSegFrame& seg,
                         LabeledCloud& out, int stride = 1,
                         bool require_label = false);

} // namespace kist
