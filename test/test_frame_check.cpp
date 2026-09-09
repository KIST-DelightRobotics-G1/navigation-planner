// Cloud <-> IMU frame check. The Mid-360 is mounted UPSIDE DOWN; if the utlidar
// relay re-orients the cloud but passes the IMU raw (or vice versa), the cloud and
// IMU end up in different frames -> gravity + yaw get mis-mapped, which would show
// as z-drift (stationary) and map-doubling on rotation. This probe reads both
// streams and reports whether they AGREE on which way is down:
//   - IMU "up" = normalize(mean stationary accel)   (a resting accel reads +g up)
//   - cloud: where the floor sits (a dense z band) + z stats
// If the floor is on the same side as IMU-down -> aligned (extrinsic ~identity ok).
// If opposite -> the cloud is flipped vs the IMU -> set ext_R to that rotation.
//   ./test_frame_check [config.yaml]   (keep the robot STILL)

#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/Imu_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ---- IMU accumulation ----
static std::mutex g_mtx;
static double g_ax = 0, g_ay = 0, g_az = 0;
static long   g_imu_n = 0;

// ---- cloud result (computed once) ----
static std::atomic<bool> g_cloud_done{false};
static double g_zmin = 1e9, g_zmax = -1e9, g_zmean = 0;
static double g_xmean = 0, g_ymean = 0;
static std::size_t g_npts = 0;
static std::vector<int> g_zhist;             // 0.5 m bins over [-6, +6]
static constexpr double kBin = 0.5, kLo = -6.0;

static int zbin(double z) {
    int b = int((z - kLo) / kBin);
    if (b < 0) b = 0; if (b >= 24) b = 23;
    return b;
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string cfg = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(cfg);
    const auto& root = kist::Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;

    g_zhist.assign(24, 0);

    unitree::robot::ChannelFactory::Instance()->Init(domain, "");

    static unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::Imu_> imu_sub("rt/utlidar/imu_livox_mid360");
    imu_sub.InitChannel([](const void* m) {
        const auto& imu = *static_cast<const sensor_msgs::msg::dds_::Imu_*>(m);
        const auto& a = imu.linear_acceleration();
        std::lock_guard<std::mutex> lk(g_mtx);
        g_ax += a.x(); g_ay += a.y(); g_az += a.z(); ++g_imu_n;
    }, 20);

    static unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_> cloud_sub("rt/utlidar/cloud_livox_mid360");
    cloud_sub.InitChannel([](const void* m) {
        if (g_cloud_done.load()) return;
        const auto& c = *static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(m);
        uint32_t ox = 0, oy = 0, oz = 0; bool fx = false, fy = false, fz = false;
        for (const auto& f : c.fields()) {
            if (f.datatype() != 7) continue;
            if (f.name() == "x") { ox = f.offset(); fx = true; }
            else if (f.name() == "y") { oy = f.offset(); fy = true; }
            else if (f.name() == "z") { oz = f.offset(); fz = true; }
        }
        if (!(fx && fy && fz)) return;
        const uint8_t* blob = c.data().data();
        const std::size_t step = c.point_step();
        const std::size_t n = c.point_step() ? c.data().size() / c.point_step() : 0;
        double zsum = 0, xsum = 0, ysum = 0; std::size_t kept = 0;
        for (std::size_t i = 0; i < n; ++i) {
            float x, y, z;
            std::memcpy(&x, blob + i*step + ox, 4);
            std::memcpy(&y, blob + i*step + oy, 4);
            std::memcpy(&z, blob + i*step + oz, 4);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            xsum += x; ysum += y; zsum += z;
            if (z < g_zmin) g_zmin = z; if (z > g_zmax) g_zmax = z;
            g_zhist[zbin(z)]++;
            ++kept;
        }
        if (kept) { g_zmean = zsum/kept; g_xmean = xsum/kept; g_ymean = ysum/kept; g_npts = kept; }
        g_cloud_done.store(true);
    }, 1);

    std::printf("collecting (keep the robot STILL)...\n");
    for (int i = 0; i < 30 && !(g_cloud_done.load() && g_imu_n > 100); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ---- IMU ----
    double ax, ay, az; long nn;
    { std::lock_guard<std::mutex> lk(g_mtx); ax = g_ax; ay = g_ay; az = g_az; nn = g_imu_n; }
    if (nn == 0) { std::printf("no IMU received\n"); return 1; }
    ax /= nn; ay /= nn; az /= nn;
    const double an = std::sqrt(ax*ax + ay*ay + az*az);
    // resting accelerometer reads +g along "up"
    std::printf("\nIMU mean accel = (% .3f % .3f % .3f)  |a|=%.3f\n", ax, ay, az, an);
    std::printf("IMU up   = (% .2f % .2f % .2f)  (normalize accel)\n", ax/an, ay/an, az/an);
    std::printf("IMU down = (% .2f % .2f % .2f)\n", -ax/an, -ay/an, -az/an);

    // ---- cloud ----
    if (!g_cloud_done.load()) { std::printf("no cloud received\n"); return 1; }
    std::printf("\ncloud: %zu pts  mean=(% .2f % .2f % .2f)  z range [% .2f, % .2f]\n",
                g_npts, g_xmean, g_ymean, g_zmean, g_zmin, g_zmax);
    std::printf("z histogram (0.5 m bins), find the dense floor band:\n");
    for (int b = 0; b < 24; ++b) {
        if (g_zhist[b] == 0) continue;
        const double zlo = kLo + b*kBin;
        std::printf("  z[% .1f,% .1f): %6d %s\n", zlo, zlo+kBin, g_zhist[b],
                    std::string(std::min(60, g_zhist[b] / 200), '#').c_str());
    }

    std::printf("\n-> IMU down axis is where accel is NEGATIVE. The floor (dense z band) "
                "should be on the IMU-DOWN side.\n"
                "   floor on IMU-down side  => cloud & IMU AGREE (ext_R ~ identity ok).\n"
                "   floor on IMU-up side    => cloud is FLIPPED vs IMU -> set ext_R.\n");
    return 0;
}
