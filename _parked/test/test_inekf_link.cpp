// Smoke test for the vendored invariant-ekf: confirms it compiles + links with
// Eigen only (no Boost for the core) and that the API we build the wrapper on is
// present — construct a filter, propagate one IMU sample, read the state back.
//   ./test_inekf_link   -> exit 0 on success

#include "InEKF.h"
#include "RobotState.h"
#include "NoiseParams.h"

#include <Eigen/Dense>
#include <cstdio>

int main() {
    inekf::RobotState  state;                 // default: X=I(5x5), P=I(15x15)
    inekf::NoiseParams noise;                 // default noise
    inekf::InEKF       filter(state, noise);

    // One propagation: gyro = 0, accel = specific force of a stationary IMU (+g up).
    Eigen::Matrix<double, 6, 1> imu;
    imu << 0.0, 0.0, 0.0,  0.0, 0.0, 9.81;
    filter.Propagate(imu, 0.005);

    inekf::RobotState s = filter.getState();  // getters are non-const in this lib
    const Eigen::Vector3d p = s.getPosition();
    const Eigen::Vector3d v = s.getVelocity();
    std::printf("[inekf smoke] pos=(%.4f %.4f %.4f) vel=(%.4f %.4f %.4f)\n",
                p.x(), p.y(), p.z(), v.x(), v.y(), v.z());

    const bool finite = p.allFinite() && v.allFinite();
    std::printf("%s\n", finite ? "inekf link OK" : "inekf link produced non-finite state");
    return finite ? 0 : 1;
}
