// sensor_extrinsics loader tests — YAML load, name->FrameId, translation preserved,
// RPY->quaternion, and the two rejections (bad frame name, duplicate edge).
//   ./test_sensor_extrinsics   -> exit 0 all pass, 1 on failure

#include "frames/sensor_extrinsics.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

using kist::FrameId;
using kist::SensorExtrinsics;
using kist::load_sensor_extrinsics;

static int g_fail = 0;
static void check(bool ok, const char* name) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

// Write text to a temp file and return its path.
static std::string write_tmp(const char* stem, const std::string& body) {
    std::string path = std::string("/tmp/") + stem + ".yaml";
    std::ofstream(path) << body;
    return path;
}

int main() {
    // --- happy path: two edges, one with a 90 deg yaw ------------------------
    const std::string good =
        "transforms:\n"
        "  - parent: torso\n"
        "    child: lidar\n"
        "    translation: { x: 0.10, y: -0.20, z: 0.70 }\n"
        "    rotation_rpy: { roll: 0.0, pitch: 0.0, yaw: 1.5707963267948966 }\n"
        "  - parent: d455\n"
        "    child: depth_optical\n"
        "    translation: { x: 0.0, y: 0.0, z: 0.0 }\n"
        "    rotation_rpy: { roll: 0.0, pitch: 0.0, yaw: 0.0 }\n";
    const std::string good_path = write_tmp("extr_good", good);

    std::string err;
    auto ext = load_sensor_extrinsics(good_path, &err);
    check(ext.has_value(), "YAML load succeeds");
    if (ext) {
        check(ext->transforms.size() == 2, "two edges parsed");

        const auto& e0 = ext->transforms[0];
        check(e0.parent == FrameId::Torso && e0.child == FrameId::Lidar,
              "parent/child string -> FrameId");

        const auto& t = e0.T_parent_child.translation;
        check(std::abs(t.x() - 0.10) < 1e-9 &&
              std::abs(t.y() + 0.20) < 1e-9 &&
              std::abs(t.z() - 0.70) < 1e-9, "translation preserved");

        // 90 deg yaw: rotating +X (forward) should give +Y (left).
        const Eigen::Vector3d fwd = e0.T_parent_child.rotateVector(
            Eigen::Vector3d::UnitX());
        check(std::abs(fwd.x()) < 1e-6 &&
              std::abs(fwd.y() - 1.0) < 1e-6 &&
              std::abs(fwd.z()) < 1e-6, "RPY yaw -> quaternion");

        const auto& e1 = ext->transforms[1];
        check(e1.parent == FrameId::D455 && e1.child == FrameId::DepthOptical,
              "second edge d455 -> depth_optical");
    }

    // --- reject unknown frame name ------------------------------------------
    const std::string bad_name =
        "transforms:\n"
        "  - parent: torso\n"
        "    child: elbow\n"                      // not a real frame
        "    translation: { x: 0, y: 0, z: 0 }\n"
        "    rotation_rpy: { roll: 0, pitch: 0, yaw: 0 }\n";
    auto r_name = load_sensor_extrinsics(write_tmp("extr_badname", bad_name), &err);
    check(!r_name.has_value(), "reject unknown frame name");

    // --- reject duplicate parent->child edge --------------------------------
    const std::string dup =
        "transforms:\n"
        "  - parent: torso\n"
        "    child: lidar\n"
        "    translation: { x: 0, y: 0, z: 0 }\n"
        "    rotation_rpy: { roll: 0, pitch: 0, yaw: 0 }\n"
        "  - parent: torso\n"
        "    child: lidar\n"
        "    translation: { x: 1, y: 0, z: 0 }\n"
        "    rotation_rpy: { roll: 0, pitch: 0, yaw: 0 }\n";
    auto r_dup = load_sensor_extrinsics(write_tmp("extr_dup", dup), &err);
    check(!r_dup.has_value(), "reject duplicate edge");

    // --- reject missing file -------------------------------------------------
    auto r_missing = load_sensor_extrinsics("/tmp/does_not_exist_xyz.yaml", &err);
    check(!r_missing.has_value(), "reject missing file");

    std::printf("%s (%d failure%s)\n",
                g_fail ? "FAILED" : "all sensor_extrinsics tests passed",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
