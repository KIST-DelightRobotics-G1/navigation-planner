// map_recorder — build a prior PCD map from the LIO engine's registered scans, ROS-free.
//
// FAST-LIO's built-in pcd_save is dead in our branch, so we record the map ourselves: we
// already consume /cloud_registered_1 (registered = odom frame) over DDS via LioReceiver.
// This tool accumulates every scan into a voxel grid (bounded size) and, on Ctrl+C, writes
// the centroid cloud as an ASCII PCD (x y z intensity) — the prior map for localization.
// The voxel-accumulation logic is the seed of the runtime Local Submap Builder (reused).
//
//   ./build/map_recorder [out.pcd=lio_map.pcd] [config=config/config.yaml]   (VOXEL=0.05 env)
//   -> drive the whole environment slowly, then Ctrl+C to save.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "lio/lio_receiver.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <unordered_map>

using namespace kist;

namespace {
std::atomic<bool> g_stop{false};

// Pack a voxel index into one int64 key (21 bits/axis, biased non-negative). At leaf 0.05 m
// the addressable range is ±~52 km — far beyond any indoor/odom extent.
inline int64_t voxel_key(float x, float y, float z, float inv_leaf) {
    const int64_t bx = int64_t(std::floor(x * inv_leaf)) + (1 << 20);
    const int64_t by = int64_t(std::floor(y * inv_leaf)) + (1 << 20);
    const int64_t bz = int64_t(std::floor(z * inv_leaf)) + (1 << 20);
    return (bx & 0x1FFFFF) | ((by & 0x1FFFFF) << 21) | ((bz & 0x1FFFFF) << 42);
}

struct Vox { double sx = 0, sy = 0, sz = 0, si = 0; uint32_t n = 0; };
}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string out_path = (argc >= 2) ? argv[1] : "lio_map.pcd";
    const std::string cfg_path = (argc >= 3) ? argv[2] : "config/config.yaml";
    const float leaf = [] { const char* v = std::getenv("VOXEL"); return v ? float(std::atof(v)) : 0.05f; }();
    const float inv_leaf = 1.0f / leaf;

    Config::instance().load(cfg_path);
    const auto& root = Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!apply_dds_config(root)) return 1;
    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    LioReceiver rx;                                  // odom_hook unused: we only need the cloud
    if (!rx.start(domain)) { std::fprintf(stderr, "[map_recorder] LIO receiver failed\n"); return 1; }
    std::printf("[map_recorder] recording rt/cloud_registered_1 -> %s (voxel %.3f m). "
                "Drive the environment, then Ctrl+C to save.\n", out_path.c_str(), leaf);

    std::unordered_map<int64_t, Vox> grid;
    grid.reserve(1u << 20);
    int64_t     last_stamp = 0;
    uint64_t    scans = 0, raw_points = 0;
    auto        last_print = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto cloud = rx.cloud_buf.GetData();
        if (!cloud || cloud->stamp_ns == last_stamp) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        last_stamp = cloud->stamp_ns;
        ++scans;
        const bool has_i = cloud->has_intensity();
        const std::size_t np = cloud->point_count();
        for (std::size_t i = 0; i < np; ++i) {
            const float x = cloud->xyz[3*i], y = cloud->xyz[3*i+1], z = cloud->xyz[3*i+2];
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            Vox& v = grid[voxel_key(x, y, z, inv_leaf)];
            v.sx += x; v.sy += y; v.sz += z; v.si += has_i ? cloud->intensity[i] : 0.0; ++v.n;
        }
        raw_points += np;

        const auto now = std::chrono::steady_clock::now();
        if (now - last_print >= std::chrono::milliseconds(1000)) {
            last_print = now;
            std::printf("[map_recorder] scans=%llu raw=%lluk voxels=%zu\n",
                        (unsigned long long)scans, (unsigned long long)(raw_points/1000), grid.size());
        }
    }
    rx.stop();

    if (grid.empty()) { std::fprintf(stderr, "[map_recorder] no points received — nothing saved.\n"); return 1; }

    FILE* f = std::fopen(out_path.c_str(), "w");
    if (!f) { std::fprintf(stderr, "[map_recorder] cannot open %s for write\n", out_path.c_str()); return 1; }
    const std::size_t N = grid.size();
    std::fprintf(f,
        "# .PCD v0.7 - prior LIO map (odom frame, voxel %.3f m)\n"
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        "WIDTH %zu\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %zu\nDATA ascii\n",
        leaf, N, N);
    for (const auto& [k, v] : grid) {
        const double inv = 1.0 / v.n;
        std::fprintf(f, "%.4f %.4f %.4f %.1f\n", v.sx*inv, v.sy*inv, v.sz*inv, v.si*inv);
    }
    std::fclose(f);
    std::printf("[map_recorder] saved %zu points (from %llu scans, %lluk raw) -> %s\n",
                N, (unsigned long long)scans, (unsigned long long)(raw_points/1000), out_path.c_str());
    return 0;
}
