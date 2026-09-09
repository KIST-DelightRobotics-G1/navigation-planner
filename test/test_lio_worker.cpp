// LioWorker smoke — feeds a few synthetic Mid-360 frames (a planar wall) + a
// stationary IMU stream through the ported FAST-LIO core, and prints the pose it
// returns. Not an accuracy test (synthetic scene); it proves the whole worker
// (convert -> IMU propagate/undistort -> ikd-Tree map -> ESIKF update) is wired and
// runs without crashing, through the PIMPL facade (no PCL/FAST-LIO in this TU).
//   ./test_lio_worker -> exit 0

#include "lio/lio_worker.hpp"

#include <cmath>
#include <cstdio>

int main() {
    kist::LioConfig cfg;
    cfg.point_filter_num = 1;
    cfg.blind = 0.1;
    kist::LioWorker lio(cfg);

    for (int f = 0; f < 6; ++f) {
        kist::UnitreePointCloud frame;
        frame.stamp_ns = int64_t(f) * 100000000LL;    // 10 Hz frames

        // A wall at x = 3 m: a grid in (y, z), with per-point time across ~90 ms.
        int idx = 0;
        for (int iy = -10; iy <= 10; ++iy) {
            for (int iz = -10; iz <= 10; ++iz) {
                frame.xyz.push_back(3.0f);
                frame.xyz.push_back(0.1f * iy);
                frame.xyz.push_back(0.1f * iz);
                frame.intensity.push_back(50.0f);
                frame.time.push_back(float(idx) * 2.0e5f);   // ns offsets, ~90ms span
                ++idx;
            }
        }

        std::deque<kist::ImuSample> imu;
        for (int j = 0; j < 20; ++j) {                 // 200 Hz stationary IMU
            kist::ImuSample s;
            s.stamp_ns = frame.stamp_ns + int64_t(j) * 5000000LL;
            s.accel = Eigen::Vector3d(0, 0, 9.81);
            imu.push_back(s);
        }

        const kist::LioPose p = lio.process(frame, imu);
        std::printf("frame %d: valid=%d pos=(% .3f % .3f % .3f) vel=(% .3f % .3f % .3f)\n",
                    f, int(p.valid),
                    p.position.x(), p.position.y(), p.position.z(),
                    p.linear_velocity.x(), p.linear_velocity.y(), p.linear_velocity.z());
    }

    std::printf("lio worker OK (ran without crashing)\n");
    return 0;
}
