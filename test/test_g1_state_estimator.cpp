// G1StateEstimator standalone tests, no robot/DDS — synthetic input into the InEKF
// wrapper. These check the wiring/plumbing (propagation integrates, contact
// correction pins a stationary robot, state stays finite), NOT filter tuning; the
// real stationary/shake/walk validation runs on-robot with live data later.
//   ./test_g1_state_estimator   -> exit 0 all pass, 1 on failure

#include "state_estimation/g1_state_estimator.hpp"

#include <cmath>
#include <cstdio>

using kist::EstimatorState;
using kist::FootContactSample;
using kist::G1StateEstimator;
using kist::ImuSample;
using kist::LegJointSample;

static int g_fail = 0;
static void check(bool ok, const char* name) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

int main() {
    const double   hz    = 200.0;
    const int64_t  dt_ns = int64_t(1e9 / hz);

    // ---- stationary robot stays put -----------------------------------------
    {
        G1StateEstimator est;
        est.reset(EstimatorState{});             // origin, identity, zero velocity

        // Stationary IMU: gyro 0, specific force = +g up (cancels gravity inside).
        ImuSample imu; imu.accel = Eigen::Vector3d(0, 0, 9.81);
        LegJointSample joints;                   // nominal (all-zero) stance
        FootContactSample contact; contact.left_contact = true; contact.right_contact = true;

        int64_t t = 0;
        for (int i = 0; i < 400; ++i) {          // 2 s
            t += dt_ns;
            imu.stamp_ns = t;
            est.propagate(imu);
            if (i % 4 == 0) {                    // 50 Hz kinematic correction
                joints.stamp_ns = t; contact.stamp_ns = t;
                est.updateKinematics(joints, contact);
            }
        }
        const EstimatorState s = est.state();
        check(s.pose.position.allFinite() && s.linear_velocity.allFinite(),
              "stationary: state finite");
        check(s.pose.position.norm() < 0.05, "stationary: position stays near origin");
        check(s.linear_velocity.norm() < 0.05, "stationary: velocity stays near zero");
        check(s.pose.stamp_ns == t, "state stamp tracks latest input");
    }

    // ---- pure propagation integrates a nonzero initial velocity -------------
    {
        G1StateEstimator est;
        EstimatorState init;
        init.linear_velocity = Eigen::Vector3d(1.0, 0, 0);   // 1 m/s forward
        est.reset(init);

        ImuSample imu; imu.accel = Eigen::Vector3d(0, 0, 9.81);   // zero net accel
        int64_t t = 0;
        for (int i = 0; i < 200; ++i) {          // 1 s, no contacts -> no correction
            t += dt_ns;
            imu.stamp_ns = t;
            est.propagate(imu);
        }
        const EstimatorState s = est.state();
        // ~1 s at 1 m/s -> ~1 m forward; velocity unchanged.
        check(std::abs(s.pose.position.x() - 1.0) < 0.05, "propagation moves ~1 m at 1 m/s");
        check(std::abs(s.linear_velocity.x() - 1.0) < 0.02, "velocity preserved under zero accel");
    }

    std::printf("%s (%d failure%s)\n",
                g_fail ? "FAILED" : "all g1_state_estimator tests passed",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
