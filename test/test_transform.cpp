// Transform unit tests. A wrong SE(3) op gives plausible-looking but globally
// broken maps, so pin the convention (T_A_B, FLU) down with concrete cases.
//   ./test_transform   -> exit 0 all pass, 1 on any failure

#include "transforms/transform.hpp"

#include <cmath>
#include <cstdio>

using kist::Transform;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using Eigen::Vector3d;

static int g_fail = 0;
static void check(bool ok, const char* name) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}
static bool close(const Vector3d& a, const Vector3d& b, double eps = 1e-9) {
    return (a - b).norm() < eps;
}

int main() {
    // 1. Identity: T * p == p
    check(close(Transform::Identity() * Vector3d(1, 2, 3), Vector3d(1, 2, 3)), "identity");

    // 2. Translation: t=(1,0,0), p_B=0 -> p_A=(1,0,0)
    {
        Transform T;
        T.translation = Vector3d(1, 0, 0);
        check(close(T * Vector3d(0, 0, 0), Vector3d(1, 0, 0)), "translation");
    }

    // 3. Rotation yaw +90 (FLU): p_B=(1,0,0) -> p_A=(0,1,0)
    {
        Transform T;
        T.rotation = Quaterniond(AngleAxisd(M_PI / 2, Vector3d::UnitZ()));
        check(close(T * Vector3d(1, 0, 0), Vector3d(0, 1, 0)), "rotation yaw+90 (point)");
        check(close(T.rotateVector(Vector3d(1, 0, 0)), Vector3d(0, 1, 0)), "rotation yaw+90 (vector)");
    }

    // 4. Composition + inverse
    {
        Transform T_A_B;
        T_A_B.rotation    = Quaterniond(AngleAxisd(0.3, Vector3d::UnitZ()));
        T_A_B.translation = Vector3d(1, 2, 0);
        Transform T_B_C;
        T_B_C.rotation    = Quaterniond(AngleAxisd(0.7, Vector3d::UnitX()));
        T_B_C.translation = Vector3d(0, 0, 3);

        const Vector3d p_C(0.5, -0.4, 1.2);
        check(close((T_A_B * T_B_C) * p_C, T_A_B * (T_B_C * p_C)), "composition == chained");

        const Vector3d p(0.1, 0.2, 0.3);
        check(close(T_A_B.inverse() * (T_A_B * p), p), "inverse round-trip");
    }

    std::printf("%s (%d failure%s)\n", g_fail ? "FAILED" : "all transform tests passed",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
