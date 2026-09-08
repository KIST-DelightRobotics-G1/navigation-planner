// G1 leg FK (pelvis -> ankle_roll link) verification. There is no legacy foot FK to
// cross-check against (unlike the waist), so these are INDEPENDENT sanity checks:
//   - q=0 net rotation is exactly identity (the hip_roll -0.1749 and knee +0.1749
//     fixed pitch offsets cancel), which catches a dropped/mis-signed rpy offset;
//   - q=0 geometry is physically plausible (foot ~0.75 m below, correct side);
//   - left/right MIRROR symmetry (right params are a true y-mirror of left), which
//     catches a left/right parameter-entry bug;
//   - random angles stay finite.
// The DEFINITIVE cross-check (against Pinocchio / the URDF / measured foot poses)
// is run externally — see the notes handed back with this file.
//   ./test_g1_foot_fk   -> exit 0 all pass, 1 on failure

#include "kinematics/g1_kinematics.hpp"

#include <cmath>
#include <cstdio>
#include <random>

using kist::G1Kinematics;
using kist::LegJoints;
using kist::Transform;

static int g_fail = 0;
static void check(bool ok, const char* name) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

// Right joints that produce the sagittal mirror of the left posture `q`: angles
// about X (roll) and Z (yaw) flip sign; angles about Y (pitch/knee) stay.
static LegJoints mirror(const LegJoints& q) {
    LegJoints m;
    m.hip_pitch   =  q.hip_pitch;
    m.hip_roll    = -q.hip_roll;
    m.hip_yaw     = -q.hip_yaw;
    m.knee        =  q.knee;
    m.ankle_pitch =  q.ankle_pitch;
    m.ankle_roll  = -q.ankle_roll;
    return m;
}

int main() {
    // ---- q = 0 ---------------------------------------------------------------
    const LegJoints zero;
    const Transform L0 = G1Kinematics::pelvisToLeftFoot(zero);
    const Transform R0 = G1Kinematics::pelvisToRightFoot(zero);

    check(L0.rotation.angularDistance(Eigen::Quaterniond::Identity()) < 1e-9 &&
          R0.rotation.angularDistance(Eigen::Quaterniond::Identity()) < 1e-9,
          "q=0 net rotation is identity (rpy offsets cancel)");

    check(L0.translation.z() < -0.65 && L0.translation.z() > -0.85 &&
          std::abs(L0.translation.x()) < 0.15,
          "q=0 foot ~0.75 m below pelvis, small x");

    check(L0.translation.y() > 0.05 && R0.translation.y() < -0.05,
          "q=0 left foot on +y, right foot on -y");

    // ---- mirror symmetry: left(q) == y-mirror of right(mirror(q)) -----------
    std::mt19937 rng(2024);
    std::uniform_real_distribution<double> d(-0.6, 0.6);
    bool mirror_ok = true, finite_ok = true;
    for (int i = 0; i < 500; ++i) {
        LegJoints q;
        q.hip_pitch = d(rng); q.hip_roll = d(rng); q.hip_yaw = d(rng);
        q.knee = d(rng); q.ankle_pitch = d(rng); q.ankle_roll = d(rng);

        const Transform L = G1Kinematics::pelvisToLeftFoot(q);
        const Transform R = G1Kinematics::pelvisToRightFoot(mirror(q));

        if (std::abs(L.translation.x() - R.translation.x()) > 1e-9 ||
            std::abs(L.translation.y() + R.translation.y()) > 1e-9 ||
            std::abs(L.translation.z() - R.translation.z()) > 1e-9)
            mirror_ok = false;

        if (!L.translation.allFinite() || !R.translation.allFinite())
            finite_ok = false;
    }
    check(mirror_ok, "left/right mirror symmetry over 500 random postures");
    check(finite_ok, "random postures stay finite");

    std::printf("%s (%d failure%s)\n",
                g_fail ? "FAILED" : "all foot FK tests passed",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
