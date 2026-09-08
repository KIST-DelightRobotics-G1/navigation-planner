// TransformTree tests — static+dynamic edge composition, time interpolation on the
// deskew hot path lookupTransform(Odom, Lidar, t), inverse direction, and the
// nullopt gaps (missing edge, out-of-range stamp).
//   ./test_transform_tree   -> exit 0 all pass, 1 on failure

#include "transforms/transform_tree.hpp"

#include <cmath>
#include <cstdio>

using kist::FrameId;
using kist::Transform;
using kist::TransformTree;

static int g_fail = 0;
static void check(bool ok, const char* name) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

static Transform xlate(double x, double y, double z) {
    Transform t;
    t.translation = Eigen::Vector3d(x, y, z);
    return t;
}
static Transform yaw(double rad) {
    Transform t;
    t.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ()));
    return t;
}

int main() {
    TransformTree tree;

    // Static mount: lidar 0.1 m forward of torso.
    tree.setStaticTransform(FrameId::Torso, FrameId::Lidar, xlate(0.10, 0, 0));

    // Dynamic: torso fixed to pelvis (no waist motion), pelvis slides in odom.
    tree.updateTransform(FrameId::Pelvis, FrameId::Torso, xlate(0, 0, 0), 1000);
    tree.updateTransform(FrameId::Pelvis, FrameId::Torso, xlate(0, 0, 0), 2000);
    tree.updateTransform(FrameId::Odom, FrameId::Pelvis, xlate(0, 0, 0), 1000);
    tree.updateTransform(FrameId::Odom, FrameId::Pelvis, xlate(2, 0, 0), 2000);

    // T_odom_lidar(1500) = (1,0,0) * I * (0.1,0,0) = (1.1, 0, 0).
    auto T = tree.lookupTransform(FrameId::Odom, FrameId::Lidar, 1500);
    check(T.has_value(), "lookup Odom<-Lidar succeeds");
    if (T) {
        const auto& t = T->translation;
        check(std::abs(t.x() - 1.10) < 1e-9 &&
              std::abs(t.y()) < 1e-9 && std::abs(t.z()) < 1e-9,
              "static+dynamic translation compose + interp");
    }

    // Rotation interpolation: pelvis yaw 0 -> 90 deg, so at 1500 -> 45 deg. The
    // 0.1 m forward lidar offset rotates to (0.1 cos45, 0.1 sin45, 0).
    TransformTree tree2;
    tree2.setStaticTransform(FrameId::Torso, FrameId::Lidar, xlate(0.10, 0, 0));
    tree2.updateTransform(FrameId::Pelvis, FrameId::Torso, xlate(0, 0, 0), 1000);
    tree2.updateTransform(FrameId::Pelvis, FrameId::Torso, xlate(0, 0, 0), 2000);
    tree2.updateTransform(FrameId::Odom, FrameId::Pelvis, yaw(0.0), 1000);
    tree2.updateTransform(FrameId::Odom, FrameId::Pelvis, yaw(M_PI / 2), 2000);
    auto R = tree2.lookupTransform(FrameId::Odom, FrameId::Lidar, 1500);
    check(R.has_value(), "lookup with rotating pelvis succeeds");
    if (R) {
        const double c = 0.10 * std::cos(M_PI / 4);
        check(std::abs(R->translation.x() - c) < 1e-6 &&
              std::abs(R->translation.y() - c) < 1e-6,
              "SLERP rotation applied to child offset");
    }

    // Inverse direction: T_lidar_odom == inverse(T_odom_lidar).
    auto inv = tree.lookupTransform(FrameId::Lidar, FrameId::Odom, 1500);
    check(inv.has_value() &&
          std::abs(inv->translation.x() + 1.10) < 1e-9, "inverse direction lookup");

    // Missing edge above the LCA is not needed: Odom<-Lidar works without map->odom.
    // But asking for Map<-Lidar (map->odom unset) must fail.
    auto miss = tree.lookupTransform(FrameId::Map, FrameId::Lidar, 1500);
    check(!miss.has_value(), "missing edge (map->odom) -> nullopt");

    // Out-of-range stamp on a dynamic edge -> nullopt (no extrapolation).
    auto oor = tree.lookupTransform(FrameId::Odom, FrameId::Lidar, 5000);
    check(!oor.has_value(), "out-of-range stamp -> nullopt");

    // Identity: same frame -> identity transform.
    auto id = tree.lookupTransform(FrameId::Lidar, FrameId::Lidar, 1500);
    check(id.has_value() && id->translation.norm() < 1e-12, "same-frame identity");

    std::printf("%s (%d failure%s)\n",
                g_fail ? "FAILED" : "all transform_tree tests passed",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
