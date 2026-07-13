// Deterministic probe of the PointCloud2 decoder (no robot needed).
//
// Builds synthetic DDS messages and checks: field-offset lookup with
// extra fields present, point extraction across point_step strides,
// non-finite point removal, stamp/frame passthrough, and rejection of
// clouds without x/y/z.

#include "lidar/pointcloud2_decode.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

using namespace kist;
using sensor_msgs::msg::dds_::PointCloud2_;
using sensor_msgs::msg::dds_::PointField_;

static int g_failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%-46s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

static PointField_ field(const char* name, uint32_t offset) {
    PointField_ f;
    f.name(name);
    f.offset(offset);
    f.datatype(7);  // FLOAT32
    f.count(1);
    return f;
}

int main() {
    // 4 points, 16-byte stride (x y z intensity), one NaN point
    PointCloud2_ msg;
    msg.header().frame_id("lidar_link");
    msg.header().stamp().sec(12);
    msg.header().stamp().nanosec(500000000);
    msg.height(1);
    msg.width(4);
    msg.fields({field("x", 0), field("y", 4), field("z", 8), field("intensity", 12)});
    msg.point_step(16);
    msg.row_step(64);

    float pts[4][4] = {
        {1.0f, 2.0f, 3.0f, 99.0f},
        {std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 99.0f},
        {-4.5f, 0.25f, 7.75f, 99.0f},
        {0.0f, -1.0f, 0.5f, 99.0f},
    };
    std::vector<uint8_t> blob(sizeof(pts));
    std::memcpy(blob.data(), pts, sizeof(pts));
    msg.data(blob);

    PointCloudXYZFrame frame;
    check("decode returns true", decode_pointcloud2(msg, frame));
    check("NaN point dropped (3 of 4 kept)", frame.point_count() == 3);
    check("stamp = 12.5s in ns", frame.stamp_ns == 12500000000LL);
    check("frame_id passthrough", frame.frame_id == "lidar_link");
    check("point 0 exact",
          frame.xyz[0] == 1.0f && frame.xyz[1] == 2.0f && frame.xyz[2] == 3.0f);
    check("point 2 exact (post-NaN indexing)",
          frame.xyz[3] == -4.5f && frame.xyz[4] == 0.25f && frame.xyz[5] == 7.75f);

    // x/y/z missing -> rejected
    PointCloud2_ bad = msg;
    bad.fields({field("intensity", 12)});
    PointCloudXYZFrame unused;
    check("cloud without x/y/z rejected", !decode_pointcloud2(bad, unused));

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
