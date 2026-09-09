// Live LIO runner + registered-cloud dump. Starts the Mid-360 readers, runs the
// ported FAST-LIO worker, prints T_odom_lidar at ~5 Hz, and ACCUMULATES the
// registered (odom-frame) clouds. On Ctrl+C it writes them to an ASCII .pcd so the
// stabilization can be checked visually (CloudCompare / pcl_viewer): a real static
// wall should show up THIN if the LIO motion compensation works, THICK/smeared if
// raw sensor-frame points had been stacked instead.
//   ./test_lio_run [config.yaml] [out.pcd]

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "lio/lio_runtime.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

static std::atomic<bool> g_stop{false};

static void save_pcd(const std::string& path,
                     const std::vector<float>& xyz, const std::vector<float>& inten) {
    const std::size_t n = xyz.size() / 3;
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) { std::printf("could not open %s for writing\n", path.c_str()); return; }
    std::fprintf(f, "# .PCD v0.7 - registered LIO cloud (odom frame)\n");
    std::fprintf(f, "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
                    "TYPE F F F F\nCOUNT 1 1 1 1\n");
    std::fprintf(f, "WIDTH %zu\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %zu\nDATA ascii\n", n, n);
    for (std::size_t i = 0; i < n; ++i)
        std::fprintf(f, "%.4f %.4f %.4f %.1f\n", xyz[3*i], xyz[3*i+1], xyz[3*i+2],
                     i < inten.size() ? inten[i] : 0.0f);
    std::fclose(f);
    std::printf("saved %zu points to %s\n", n, path.c_str());
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string cfg     = (argc >= 2) ? argv[1] : "config/config.yaml";
    const std::string out_pcd = (argc >= 3) ? argv[2] : "lio_map.pcd";

    kist::Config::instance().load(cfg);
    const auto& root = kist::Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;

    std::signal(SIGINT, [](int) { g_stop = true; });

    kist::LioConfig lcfg;   // defaults incl. Mid-360 lidar<->imu extrinsic
    kist::LioRuntime rt;
    if (!rt.start(domain, lcfg)) return 1;

    std::printf("running LIO on live Mid-360 (Ctrl+C to quit + save %s)...\n"
                "stand still -> pose steady; walk -> pose tracks; the .pcd shows the map.\n",
                out_pcd.c_str());

    // Accumulate the registered clouds (odom frame) into one map for the dump.
    std::vector<float> acc_xyz, acc_intensity;
    int64_t last_stamp = -1;
    bool capped = false;
    constexpr std::size_t kMaxFloats = 6000000;   // ~2M points -> bounded .pcd

    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto r = rt.result_buf.GetData();
        if (!r) {
            std::printf("no valid result yet (processed=%llu)\n",
                        (unsigned long long)rt.frames_processed.load());
            continue;
        }
        if (r->stamp_ns != last_stamp) {         // only accumulate a fresh frame
            last_stamp = r->stamp_ns;
            if (!capped && acc_xyz.size() < kMaxFloats) {
                acc_xyz.insert(acc_xyz.end(), r->cloud.xyz.begin(), r->cloud.xyz.end());
                acc_intensity.insert(acc_intensity.end(),
                                     r->cloud.intensity.begin(), r->cloud.intensity.end());
            } else if (!capped) {
                capped = true;
                std::printf("(accumulation cap reached — dump will hold the first ~2M points)\n");
            }
        }
        const auto& q = r->pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w()*q.z() + q.x()*q.y()),
                                      1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z())) * 57.2958;
        std::printf("T_odom_lidar: pos=(% .3f % .3f % .3f) yaw=% 6.1f deg  "
                    "vel=(% .2f % .2f % .2f) | cloud=%zu pts  map=%zu pts | proc=%llu drop=%llu\n",
                    r->pose.position.x(), r->pose.position.y(), r->pose.position.z(), yaw,
                    r->pose.linear_velocity.x(), r->pose.linear_velocity.y(), r->pose.linear_velocity.z(),
                    r->cloud.point_count(), acc_xyz.size() / 3,
                    (unsigned long long)rt.frames_processed.load(),
                    (unsigned long long)rt.frames_dropped.load());
    }

    rt.stop();
    if (!acc_xyz.empty()) save_pcd(out_pcd, acc_xyz, acc_intensity);
    return 0;
}
