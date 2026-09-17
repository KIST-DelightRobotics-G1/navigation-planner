#include "recorder/map_recorder.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

namespace kist {

namespace {
// Pack a voxel index into one int64 key (21 bits/axis, biased non-negative). At leaf 0.05 m the
// addressable range is ±~52 km — far beyond any indoor/odom extent.
inline int64_t voxel_key(float x, float y, float z, float inv_leaf) {
    const int64_t bx = int64_t(std::floor(x * inv_leaf)) + (1 << 20);
    const int64_t by = int64_t(std::floor(y * inv_leaf)) + (1 << 20);
    const int64_t bz = int64_t(std::floor(z * inv_leaf)) + (1 << 20);
    return (bx & 0x1FFFFF) | ((by & 0x1FFFFF) << 21) | ((bz & 0x1FFFFF) << 42);
}
}  // namespace

bool MapRecorder::start(LioReceiver& rx, UwbReceiver* uwb, const MapRecorderConfig& cfg) {
    rx_ = &rx; uwb_ = uwb; cfg_ = cfg;
    running_ = true;
    thread_ = std::thread(&MapRecorder::run, this);
    return true;
}

void MapRecorder::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void MapRecorder::run() {
    const float inv_leaf = 1.0f / cfg_.voxel_m;
    int64_t last_stamp = 0;

    // Map-origin averaging: over the first cfg_.origin_avg_s (from the first valid sample) we average
    // distinct UWB fixes + odom poses (deduped by stamp) into one clean origin correspondence. Both
    // are averaged over the SAME window so the (UWB <-> P_B) point pair stays consistent.
    bool     acc_started = false;
    auto     acc_t0 = std::chrono::steady_clock::now();
    int64_t  last_u_stamp = 0, last_o_stamp = 0;
    double   su_x = 0, su_y = 0;                 uint32_t nu = 0;   // UWB sums
    double   sp_x = 0, sp_y = 0, s_sin = 0, s_cos = 0; uint32_t np = 0;   // P_B sums (yaw: circular)

    while (running_) {
        // Accumulate the map-origin UWB + odom pose P_B over the averaging window, then finalize once.
        if (uwb_ && !origin_have_.load()) {
            auto fx = uwb_->fix.GetData();
            auto od = rx_->odom_buf.GetData();
            if (fx && od) {
                if (!acc_started) { acc_started = true; acc_t0 = std::chrono::steady_clock::now(); }
                if (fx->stamp_ns != last_u_stamp) {           // dedupe by stamp
                    last_u_stamp = fx->stamp_ns; su_x += fx->x; su_y += fx->y; ++nu;
                }
                if (od->pose.stamp_ns != last_o_stamp) {
                    last_o_stamp = od->pose.stamp_ns;
                    sp_x += od->pose.T_parent_child.translation.x();
                    sp_y += od->pose.T_parent_child.translation.y();
                    const Eigen::Matrix3d R = od->pose.T_parent_child.rotation.toRotationMatrix();
                    const double yaw = std::atan2(R(1, 0), R(0, 0));
                    s_sin += std::sin(yaw); s_cos += std::cos(yaw); ++np;
                }
                const double elapsed =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - acc_t0).count();
                if (elapsed >= cfg_.origin_avg_s && nu > 0 && np > 0) {
                    uwb_ox_ = float(su_x / nu); uwb_oy_ = float(su_y / nu);
                    pb_x_ = float(sp_x / np);  pb_y_ = float(sp_y / np);
                    pb_yaw_ = float(std::atan2(s_sin, s_cos));   // circular mean
                    origin_have_.store(true);
                    std::printf("[MapRecorder] origin averaged over %.1fs (%u UWB, %u odom): "
                                "UWB=(%.3f, %.3f)  P_B=(%.3f, %.3f, %.1f deg)\n",
                                cfg_.origin_avg_s, nu, np, uwb_ox_, uwb_oy_, pb_x_, pb_y_, pb_yaw_ * 57.2958f);
                }
            }
        }

        auto cloud = rx_->cloud_buf.GetData();
        if (!cloud || cloud->stamp_ns == last_stamp) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        last_stamp = cloud->stamp_ns;
        ++scans_;
        const bool has_i = cloud->has_intensity();
        const std::size_t np = cloud->point_count();
        for (std::size_t i = 0; i < np; ++i) {
            const float x = cloud->xyz[3*i], y = cloud->xyz[3*i+1], z = cloud->xyz[3*i+2];
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            const int64_t key = voxel_key(x, y, z, inv_leaf);
            auto it = grid_.find(key);
            if (it == grid_.end()) { it = grid_.emplace(key, Vox{}).first; voxel_count_.fetch_add(1); }
            Vox& v = it->second;
            v.sx += x; v.sy += y; v.sz += z; v.si += has_i ? cloud->intensity[i] : 0.0; ++v.n;
        }
        raw_points_ += np;
    }
}

bool MapRecorder::save(const std::string& pcd_path) const {
    if (grid_.empty()) return false;

    FILE* f = std::fopen(pcd_path.c_str(), "w");
    if (!f) return false;
    const std::size_t N = grid_.size();
    std::fprintf(f,
        "# .PCD v0.7 - prior LIO map (odom frame, voxel %.3f m)\n"
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        "WIDTH %zu\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %zu\nDATA ascii\n",
        cfg_.voxel_m, N, N);
    for (const auto& [k, v] : grid_) {
        const double inv = 1.0 / v.n;
        std::fprintf(f, "%.4f %.4f %.4f %.1f\n", v.sx*inv, v.sy*inv, v.sz*inv, v.si*inv);
    }
    std::fclose(f);
    std::printf("[MapRecorder] saved %zu points (from %llu scans, %lluk raw) -> %s\n",
                N, (unsigned long long)scans_.load(),
                (unsigned long long)(raw_points_.load()/1000), pcd_path.c_str());

    // UWB sidecar (pcd -> .uwb): line 1 = UWB at capture, line 2 = robot map pose P_B.
    std::string sidecar = pcd_path;
    const auto dot = sidecar.rfind(".pcd");
    if (dot != std::string::npos && dot == sidecar.size() - 4) sidecar.replace(dot, 4, ".uwb");
    else sidecar += ".uwb";
    if (origin_have_.load()) {
        if (FILE* uf = std::fopen(sidecar.c_str(), "w")) {
            std::fprintf(uf, "%.4f %.4f\n", uwb_ox_, uwb_oy_);
            std::fprintf(uf, "%.4f %.4f %.4f\n", pb_x_, pb_y_, pb_yaw_);
            std::fclose(uf);
            std::printf("[MapRecorder] wrote sidecar (UWB %.3f,%.3f | P_B %.3f,%.3f,%.1fdeg) -> %s\n",
                        uwb_ox_, uwb_oy_, pb_x_, pb_y_, pb_yaw_ * 57.2958f, sidecar.c_str());
        }
    } else {
        std::printf("[MapRecorder] no UWB+odom captured — sidecar not written (deploy seed falls back to config).\n");
    }
    return true;
}

} // namespace kist
