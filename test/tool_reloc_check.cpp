// reloc_check — v0 validation for the Relocalizer (map->odom via GICP against a prior map).
//
// Loads the prior PCD, accumulates live registered scans into a rolling submap, and repeatedly
// GICP-aligns the submap to the prior from a manual seed. Prints the fitness + resulting
// T_map_odom (x, y, yaw) and dumps reloc_overlay.pcd (prior + submap-transformed-by-T_map_odom)
// so you can eyeball the overlap in pcl_viewer. This isolates FRAME/registration correctness
// before wiring map->odom into the planner.
//
//   SEED_X=.. SEED_Y=.. SEED_YAW_DEG=.. ./build/reloc_check [prior.pcd=maps/prior_map.pcd] [config]
//   -> stand/drive near the seed pose; watch fitness drop + overlap tighten. Ctrl+C to stop.
//   pcl_viewer reloc_overlay.pcd     # prior (dark) vs relocalized submap (bright) should coincide

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "lio/lio_receiver.hpp"
#include "localization/relocalizer.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

using namespace kist;

namespace {
std::atomic<bool> g_stop{false};

float envf(const char* k, float dflt) { const char* v = std::getenv(k); return v ? float(std::atof(v)) : dflt; }

Eigen::Matrix4f seed_from_xyyaw(float x, float y, float yaw) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    T(0,3) = x; T(1,3) = y;
    return T;
}
float yaw_of(const Eigen::Matrix4f& T) { return std::atan2(T(1,0), T(0,0)); }
}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string prior_path = (argc >= 2) ? argv[1] : "maps/prior_map.pcd";
    const std::string cfg_path   = (argc >= 3) ? argv[2] : "config/config.yaml";

    Config::instance().load(cfg_path);
    const auto& root = Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!apply_dds_config(root)) return 1;
    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    Relocalizer reloc;
    {   // env-tunable knobs (sweep without rebuilding)
        RelocConfig rc = reloc.config();
        rc.map_voxel_m    = envf("MAP_VOXEL",   rc.map_voxel_m);
        rc.submap_voxel_m = envf("SUB_VOXEL",   rc.submap_voxel_m);
        rc.submap_scans   = int(envf("SUB_SCANS", float(rc.submap_scans)));
        rc.gicp_max_corr_m= envf("MAX_CORR",    float(rc.gicp_max_corr_m));
        rc.fitness_max    = envf("FITNESS_MAX", float(rc.fitness_max));
        rc.max_jump_m     = envf("MAX_JUMP",    float(rc.max_jump_m));
        reloc.set_config(rc);
    }
    if (!reloc.load_prior(prior_path)) return 1;

    const float sx = envf("SEED_X", 0.f), sy = envf("SEED_Y", 0.f);
    const float syaw = envf("SEED_YAW_DEG", 0.f) * float(M_PI) / 180.f;
    reloc.seed(seed_from_xyyaw(sx, sy, syaw));
    std::printf("[reloc_check] seed T_map_odom: x=%.2f y=%.2f yaw=%.1f deg\n", sx, sy, syaw*57.2958f);

    LioReceiver rx;
    if (!rx.start(domain)) { std::fprintf(stderr, "[reloc_check] LIO receiver failed\n"); return 1; }
    std::printf("[reloc_check] receiving rt/cloud_registered_1; building submap...\n");

    int64_t last_stamp = 0;
    auto    last_align = std::chrono::steady_clock::now();
    while (!g_stop) {
        auto cloud = rx.cloud_buf.GetData();
        if (cloud && cloud->stamp_ns != last_stamp) {
            last_stamp = cloud->stamp_ns;
            reloc.add_scan(*cloud);
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - last_align >= std::chrono::milliseconds(1000)) {   // ~1 Hz align
            last_align = now;
            double fit = -1.0;
            const bool ok = reloc.align(&fit);
            const Eigen::Matrix4f T = reloc.T_map_odom();
            std::printf("[reloc_check] submap=%zu prior=%zu  align=%s fitness=%.4f  "
                        "T_map_odom x=%.2f y=%.2f yaw=%.1f\n",
                        reloc.submap_size(), reloc.prior_size(), ok ? "ACCEPT" : "reject",
                        fit, T(0,3), T(1,3), yaw_of(T)*57.2958f);
            if (ok) reloc.dump_overlay("reloc_overlay.pcd");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rx.stop();
    reloc.dump_overlay("reloc_overlay.pcd");
    std::printf("[reloc_check] stopped. pcl_viewer reloc_overlay.pcd to inspect overlap.\n");
    return 0;
}
