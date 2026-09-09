// FAST-LIO core smoke — a build/link/run gate proving the pure IKFoM ESIKF state
// and the PCL-backed ikd-Tree work ROS-free in this image, before porting
// common_lib / IMU_Processing / laserMapping into our own src/lio/.
//   ./test_fastlio_core -> exit 0

#include <omp.h>                      // esekfom.hpp uses omp_get_wtime(); include first
#include "use-ikfom.hpp"             // state_ikfom (MTK/Eigen, no ROS/PCL)
#include "ikd-Tree/ikd_Tree.h"       // KD_TREE<PointType> (PCL, no ROS)

#include <pcl/point_types.h>

#include <cstdio>

using PointT = pcl::PointXYZINormal;

int main() {
    state_ikfom s;                                   // 24-dim IKFoM manifold (pure Eigen)
    (void)s;

    // Heap + intentionally LEAKED: ~KD_TREE tears down its rebuild thread and
    // segfaults on an immediate construct/destruct. The real LIO worker keeps the
    // tree alive for the whole program (it is the persistent map), so this never
    // bites there; leaking keeps this gate clean.
    auto* tree = new KD_TREE<PointT>();
    KD_TREE<PointT>::PointVector pts;
    for (int i = 0; i < 100; ++i) {
        PointT p; p.x = 0.01f * i; p.y = 0.0f; p.z = 0.0f; p.intensity = 1.0f;
        pts.push_back(p);
    }
    tree->Build(pts);

    std::printf("fast-lio core OK: IKFoM state + ikd-Tree(size=%d) build/run ROS-free\n",
                tree->size());
    return 0;
}
