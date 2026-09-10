// FAST-LIO core smoke — build/link/run gate proving the pure IKFoM ESIKF state, the
// PCL-backed ikd-Tree, AND our de-ROS'd common_lib (MeasureGroup with kist::ImuSample,
// StatesGroup, esti_plane) all work ROS-free in this image, before porting
// IMU_Processing / laserMapping into src/lio/.
//   ./test_fastlio_core -> exit 0

#include <omp.h>                      // esekfom.hpp uses omp_get_wtime(); include first
#include "use-ikfom.hpp"             // state_ikfom (MTK/Eigen, no ROS/PCL)
#include "ikd-Tree/ikd_Tree.h"       // KD_TREE<PointType> (PCL, no ROS)
#include "lio/common_lib.h"         // de-ROS'd: MeasureGroup, StatesGroup, esti_plane
#include "lio/imu_processing.hpp"   // de-ROS'd: ImuProcess (propagation + undistort)
#include "lio/cloud_converter.hpp"  // UnitreePointCloud -> PointCloudXYZI (curvature=ms)

#include <pcl/point_types.h>

#include <cstdio>

int main() {
    state_ikfom s;                                   // 24-dim IKFoM manifold (pure Eigen)
    (void)s;

    StatesGroup states;                              // de-ROS'd legacy state
    (void)states;

    MeasureGroup meas;                               // de-ROS'd: PCL cloud + ImuSample deque
    kist::ImuSample imu;
    imu.stamp_ns = 1;
    imu.accel = Eigen::Vector3d(0, 0, 9.81);
    meas.imu.push_back(imu);
    meas.lidar_beg_time = 0.0;
    meas.lidar_end_time = 0.1;

    // De-ROS'd IMU processor: run one Process() (takes the IMU-init path with the
    // single sample) — this compiles the whole propagation/undistort chain.
    ImuProcess imu_proc;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    PointCloudXYZI::Ptr undistorted(new PointCloudXYZI());
    imu_proc.Process(meas, kf, undistorted);

    // Heap + leaked on purpose (~KD_TREE races its rebuild thread on destruct; the
    // real worker keeps the tree alive for the whole program).
    auto* tree = new KD_TREE<PointType>();
    PointVector pts;
    for (int i = 0; i < 100; ++i) {
        PointType p; p.x = 0.01f * i; p.y = 0.0f; p.z = 0.0f; p.intensity = 1.0f;
        pts.push_back(p);
    }
    tree->Build(pts);

    // cloud converter: DDS UnitreePointCloud -> FAST-LIO cloud (curvature = time ms).
    kist::UnitreePointCloud dds;
    for (int i = 0; i < 50; ++i) {
        dds.xyz.push_back(1.0f + 0.01f * i); dds.xyz.push_back(0.0f); dds.xyz.push_back(0.0f);
        dds.intensity.push_back(1.0f);
        dds.time.push_back(1.0e6f * i);      // 1 ms steps in ns -> curvature 1.0*i ms
    }
    PointCloudXYZI fl;
    kist::to_fastlio_cloud(dds, fl);
    const float last_curv = fl.empty() ? -1.0f : fl.back().curvature;

    std::printf("fast-lio core OK: IKFoM + ikd-Tree(size=%d) + common_lib + "
                "ImuProcess (meas.imu=%zu) + converter (pts=%zu, last curvature=%.1f ms) "
                "build/run ROS-free\n",
                tree->size(), meas.imu.size(), fl.size(), last_curv);
    return 0;
}
