#include "labeled_cloud/labeled_cloud.hpp"

namespace kist {

void build_labeled_cloud(const DepthFrame& d, const InstSegFrame& seg,
                         LabeledCloud& out, int stride, bool require_label) {
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
    out.instance.reserve(cap);

    for (int v = 0; v < H; v += stride) {
        const auto* row = reinterpret_cast<const uint16_t*>(
            d.data.data() + size_t(v) * d.stride_bytes);
        for (int u = 0; u < W; u += stride) {
            const uint16_t raw = row[u];
            if (raw == 0) continue;                       // no return / invalid
            const int cls  = seg.class_at(float(u), float(v));
            if (require_label && cls < 0) continue;
            const int inst = seg.instance_at(float(u), float(v));
            const float Z = raw * d.depth_scale;
            out.xyz.push_back((u - d.cx) * Z * inv_fx);
            out.xyz.push_back((v - d.cy) * Z * inv_fy);
            out.xyz.push_back(Z);
            out.label.push_back(cls < 0 ? kNoClass : uint8_t(cls));
            out.instance.push_back(int16_t(inst));
        }
    }
}

} // namespace kist
