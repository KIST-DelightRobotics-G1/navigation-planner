// Deterministic probe of the reader's decode path (no robot needed).
//
// Decode is internal to the reader (gearsonic-style), so everything is
// exercised through on_cloud_update — it never touches the DDS channel.
// Checks: field-offset lookup with extra fields present, extraction
// across point_step strides, non-finite point removal, stamp/frame_id
// passthrough, drop of clouds without x/y/z (buffer keeps the last good
// frame), and the set_processor hook running before publication.

#include "unitree/unitree_pointcloud_reader.hpp"

#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

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
    auto& reader = UnitreePointCloudReader::instance();

    // 4 points, 16-byte stride (x y z intensity), one NaN point
    PointCloud2_ msg;
    msg.header().frame_id("utlidar_lidar");
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

    // ── decode via the receive callback ─────────────────────────
    reader.on_cloud_update(&msg);
    auto cloud = reader.cloud_buf.GetData();
    check("frame published to cloud_buf", cloud != nullptr);
    check("NaN point dropped (3 of 4 kept)", cloud && cloud->point_count() == 3);
    check("stamp = 12.5s in ns", cloud && cloud->stamp_ns == 12500000000LL);
    check("frame_id passthrough", cloud && cloud->frame_id == "utlidar_lidar");
    check("point 0 exact",
          cloud && cloud->xyz[0] == 1.0f && cloud->xyz[1] == 2.0f && cloud->xyz[2] == 3.0f);
    check("point 2 exact (post-NaN indexing)",
          cloud && cloud->xyz[3] == -4.5f && cloud->xyz[4] == 0.25f && cloud->xyz[5] == 7.75f);

    // ── cloud without x/y/z: dropped, last good frame kept ──────
    PointCloud2_ bad = msg;
    bad.fields({field("intensity", 12)});
    bad.header().stamp().sec(99);
    reader.on_cloud_update(&bad);
    cloud = reader.cloud_buf.GetData();
    check("cloud without x/y/z dropped (stamp unchanged)",
          cloud && cloud->stamp_ns == 12500000000LL);

    // ── processor hook: runs between decode and publication ─────
    reader.set_processor([](UnitreePointCloud& f) {
        // keep only points with z >= 1 (drops {0,-1,0.5}), then negate x
        std::vector<float> kept;
        for (std::size_t i = 0; i < f.xyz.size(); i += 3) {
            if (f.xyz[i + 2] < 1.0f) continue;
            kept.insert(kept.end(), {-f.xyz[i], f.xyz[i + 1], f.xyz[i + 2]});
        }
        f.xyz = std::move(kept);
    });
    reader.on_cloud_update(&msg);
    reader.set_processor(nullptr);

    cloud = reader.cloud_buf.GetData();
    check("hook: processed frame published",
          cloud && cloud->xyz == std::vector<float>({-1, 2, 3, 4.5f, 0.25f, 7.75f}));

    reader.on_cloud_update(&msg);
    cloud = reader.cloud_buf.GetData();
    check("hook cleared: raw frame published",
          cloud && cloud->point_count() == 3);

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
