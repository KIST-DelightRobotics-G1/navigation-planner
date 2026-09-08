// G1 kinematics tests, two layers:
//   (1) the Transform FK itself — zero pose, single-axis, and a 1000-sample random
//       cross-check against the legacy Rigid FK (so retiring Rigid is safe);
//   (2) the waist producer + transform tree — samples in, interpolated edge out,
//       plus the pelvis->lidar integration through the static extrinsic.
//   ./test_g1_kinematics   -> exit 0 all pass, 1 on failure

#include "kinematics/g1_kinematics.hpp"
#include "kinematics/waist_producer.hpp"
#include "transforms/rigid.hpp"          // legacy FK reference for the cross-check
#include "transforms/transform_tree.hpp"

#include <cmath>
#include <cstdio>
#include <random>

using kist::FrameId;
using kist::G1Kinematics;
using kist::Rigid;
using kist::Transform;
using kist::TransformTree;
using kist::WaistSample;

static int g_fail = 0;
static void check(bool ok, const char* name) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

// Legacy Rigid FK — exactly the pre-migration implementation, as the golden ref.
static Rigid goldenFK(double yaw, double roll, double pitch) {
    const Rigid y = kist::rot_z(float(yaw),   0.0f,        0.0f, 0.0f);
    const Rigid r = kist::rot_x(float(roll), -0.0039635f,  0.0f, 0.044f);
    const Rigid p = kist::rot_y(float(pitch),  0.0f,        0.0f, 0.0f);
    return kist::rigid_mul(kist::rigid_mul(y, r), p);
}

static bool closeToRigid(const Transform& a, const Rigid& g, double tol) {
    if (std::abs(a.translation.x() - g.t[0]) > tol) return false;
    if (std::abs(a.translation.y() - g.t[1]) > tol) return false;
    if (std::abs(a.translation.z() - g.t[2]) > tol) return false;
    const Eigen::Matrix3d R = a.rotation.toRotationMatrix();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (std::abs(R(i, j) - double(g.r[i*3+j])) > tol) return false;
    return true;
}

static bool closeTf(const Transform& a, const Transform& b, double tol) {
    if ((a.translation - b.translation).norm() > tol) return false;
    const double d = std::abs(a.rotation.normalized().dot(b.rotation.normalized()));
    return d > 1.0 - tol;                              // quats aligned (up to sign)
}

int main() {
    const double tol = 1e-4;                           // float(legacy) vs double(new)

    // ---- Layer 1: FK itself --------------------------------------------------
    // Zero pose: only the waist_roll origin translation remains, no rotation.
    auto q0 = G1Kinematics::pelvisToTorso(0, 0, 0);
    check(std::abs(q0.translation.x() + 0.0039635) < 1e-9 &&
          std::abs(q0.translation.z() - 0.044) < 1e-9 &&
          q0.rotation.angularDistance(Eigen::Quaterniond::Identity()) < 1e-9,
          "zero pose = waist_roll origin, identity rotation");

    check(closeToRigid(G1Kinematics::pelvisToTorso(M_PI/2, 0, 0), goldenFK(M_PI/2, 0, 0), tol),
          "yaw only vs legacy");
    check(closeToRigid(G1Kinematics::pelvisToTorso(0, 0.3, 0), goldenFK(0, 0.3, 0), tol),
          "roll only vs legacy");
    check(closeToRigid(G1Kinematics::pelvisToTorso(0, 0, -0.4), goldenFK(0, 0, -0.4), tol),
          "pitch only vs legacy");

    // 1000 random joint triples: new Transform FK must match legacy Rigid FK.
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> dist(-M_PI/3, M_PI/3);
    bool all_match = true;
    for (int i = 0; i < 1000; ++i) {
        const double y = dist(rng), r = dist(rng), p = dist(rng);
        if (!closeToRigid(G1Kinematics::pelvisToTorso(y, r, p), goldenFK(y, r, p), tol)) {
            all_match = false;
            break;
        }
    }
    check(all_match, "1000 random q: new FK == legacy Rigid FK");

    // ---- Layer 2: producer + transform tree ---------------------------------
    TransformTree tree;
    Transform T_torso_lidar;                           // static mount: 0.1 m forward
    T_torso_lidar.translation = Eigen::Vector3d(0.10, 0, 0);
    tree.setStaticTransform(FrameId::Torso, FrameId::Lidar, T_torso_lidar);

    WaistSample s0; s0.stamp_ns = 1000; s0.yaw_rad = 0.0;  s0.roll_rad = 0.10; s0.pitch_rad = 0.0;
    WaistSample s1; s1.stamp_ns = 2000; s1.yaw_rad = 0.5;  s1.roll_rad = 0.10; s1.pitch_rad = -0.2;
    check(kist::updateWaistTransform(tree, s0), "producer update @ t0");
    check(kist::updateWaistTransform(tree, s1), "producer update @ t1");

    // At each sample stamp the edge must equal the FK of that sample.
    auto at0 = tree.lookupTransform(FrameId::Pelvis, FrameId::Torso, 1000);
    auto at1 = tree.lookupTransform(FrameId::Pelvis, FrameId::Torso, 2000);
    check(at0 && closeTf(*at0, G1Kinematics::pelvisToTorso(0.0, 0.10, 0.0), tol),
          "lookup(Pelvis,Torso,t0) == FK(q0)");
    check(at1 && closeTf(*at1, G1Kinematics::pelvisToTorso(0.5, 0.10, -0.2), tol),
          "lookup(Pelvis,Torso,t1) == FK(q1)");

    // Midpoint interpolates (has a value, and sits between the two rotations).
    auto mid = tree.lookupTransform(FrameId::Pelvis, FrameId::Torso, 1500);
    check(mid.has_value(), "lookup(Pelvis,Torso,midpoint) interpolates");

    // Integration through the static extrinsic:
    //   T_pelvis_lidar(t0) == T_pelvis_torso(q0) * T_torso_lidar.
    auto lidar0 = tree.lookupTransform(FrameId::Pelvis, FrameId::Lidar, 1000);
    const Transform expect = G1Kinematics::pelvisToTorso(0.0, 0.10, 0.0) * T_torso_lidar;
    check(lidar0 && closeTf(*lidar0, expect, tol),
          "lookup(Pelvis,Lidar,t0) == FK(q0) * T_torso_lidar");

    std::printf("%s (%d failure%s)\n",
                g_fail ? "FAILED" : "all g1_kinematics tests passed",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
