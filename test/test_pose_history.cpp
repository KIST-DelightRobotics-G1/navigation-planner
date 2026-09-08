// PoseHistory / interpolation tests — SLERP-lerp between bracketing samples, and
// the deliberate no-extrapolation policy (out-of-range query -> nullopt).
//   ./test_pose_history   -> exit 0 all pass, 1 on failure

#include "transforms/pose_history.hpp"

#include <cmath>
#include <cstdio>

using kist::Pose;
using kist::PoseHistory;

static int g_fail = 0;
static void check(bool ok, const char* name) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

int main() {
    PoseHistory h;
    Pose a; a.stamp_ns = 1000; a.position = Eigen::Vector3d(0, 0, 0);
    Pose b; b.stamp_ns = 2000; b.position = Eigen::Vector3d(2, 0, 0);
    h.push(a);
    h.push(b);

    auto mid = h.getInterpolated(1500);                 // midpoint -> x = 1
    check(mid && std::abs(mid->position.x() - 1.0) < 1e-9, "interp midpoint");

    auto front = h.getInterpolated(1000);               // exactly at front
    check(front && std::abs(front->position.x()) < 1e-9, "interp at front");

    check(!h.getInterpolated(500).has_value(),  "no extrapolation before range");
    check(!h.getInterpolated(2500).has_value(), "no extrapolation after range");

    std::printf("%s (%d failure%s)\n", g_fail ? "FAILED" : "all pose_history tests passed",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
