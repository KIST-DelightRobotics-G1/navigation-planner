// Deterministic probe of the odometry convert path (no robot needed).
//
// Builds a synthetic Odometry message and feeds it through
// on_odom_update — checks the SDK→kist field mapping (pose, quaternion,
// twist, stamp, frame_id), which is easy to get silently wrong through
// the nested accessor chains.

#include "unitree/unitree_odometry_reader.hpp"

#include <unitree/idl/ros2/Odometry_.hpp>

#include <cstdio>

using namespace kist;

static int g_failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%-46s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

int main() {
    nav_msgs::msg::dds_::Odometry_ msg;
    msg.header().frame_id("odom");
    msg.header().stamp().sec(7);
    msg.header().stamp().nanosec(250000000);

    auto& p = msg.pose().pose();
    p.position().x(1.5);
    p.position().y(-2.5);
    p.position().z(0.75);
    p.orientation().x(0.1);
    p.orientation().y(0.2);
    p.orientation().z(0.3);
    p.orientation().w(0.9);

    auto& t = msg.twist().twist();
    t.linear().x(0.4);
    t.linear().y(-0.1);
    t.linear().z(0.05);
    t.angular().x(0.01);
    t.angular().y(0.02);
    t.angular().z(-0.6);

    auto& reader = UnitreeOdometryReader::instance();
    reader.on_odom_update(&msg);

    auto odom = reader.odom_buf.GetData();
    check("sample published to odom_buf", odom != nullptr);
    if (odom) {
        check("stamp = 7.25s in ns", odom->stamp_ns == 7250000000LL);
        check("frame_id passthrough", odom->frame_id == "odom");
        check("position mapped",
              odom->px == 1.5f && odom->py == -2.5f && odom->pz == 0.75f);
        check("quaternion mapped",
              odom->qx == 0.1f && odom->qy == 0.2f &&
              odom->qz == 0.3f && odom->qw == 0.9f);
        check("linear twist mapped",
              odom->vx == 0.4f && odom->vy == -0.1f && odom->vz == 0.05f);
        check("angular twist mapped",
              odom->wx == 0.01f && odom->wy == 0.02f && odom->wz == -0.6f);
    }

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
