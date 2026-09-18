#include "localization/relocalizer.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <iostream>
#include <vector>

namespace kist {

using PointT = pcl::PointXYZI;
using Cloud  = pcl::PointCloud<PointT>;

struct Relocalizer::Impl {
    Cloud::Ptr             prior{new Cloud};       // downsampled prior map (map frame) — GICP + overlay
    Cloud::Ptr             prior_score{new Cloud}; // floor-filtered prior, for the yaw-search score
    pcl::KdTreeFLANN<PointT> prior_kdtree;         // NN over prior_score, for the global yaw-search score
    bool                   kdtree_ready = false;
    std::deque<Cloud::Ptr> scans;                  // recent registered scans (odom frame)
    Cloud::Ptr             submap{new Cloud};      // merged + downsampled submap (odom frame) — GICP
    Cloud::Ptr             submap_score{new Cloud};// floor-filtered submap, for the yaw-search score
    bool                   submap_dirty = false;
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
};

namespace {
void voxel(const Cloud::Ptr& in, Cloud::Ptr& out, float leaf) {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*out);
}

// Keep only the wall band: drop the floor (matches at any yaw) and the ceiling, leaving vertical
// structure that actually constrains yaw. Floor z is the densest z-histogram bin (frame-agnostic).
void filter_floor(const Cloud::Ptr& in, Cloud::Ptr& out, float band, float wall_top) {
    out->clear();
    if (in->empty()) return;
    float zmin = 1e9f, zmax = -1e9f;
    for (const auto& p : *in) { zmin = std::min(zmin, p.z); zmax = std::max(zmax, p.z); }
    const float bin = 0.10f;
    const int   nb  = std::max(1, int((zmax - zmin) / bin) + 1);
    std::vector<int> hist(nb, 0);
    for (const auto& p : *in) {
        int b = int((p.z - zmin) / bin);
        if (b < 0) b = 0; else if (b >= nb) b = nb - 1;
        ++hist[b];
    }
    int fb = 0;
    for (int i = 1; i < nb; ++i) if (hist[i] > hist[fb]) fb = i;
    const float z_floor = zmin + (fb + 0.5f) * bin;
    out->reserve(in->size());
    for (const auto& p : *in)
        if (p.z > z_floor + band && p.z < z_floor + wall_top) out->push_back(p);
}

Cloud::Ptr to_pcl(const LioCloud& s) {
    Cloud::Ptr c(new Cloud);
    const std::size_t n = s.point_count();
    c->reserve(n);
    const bool hi = s.has_intensity();
    for (std::size_t i = 0; i < n; ++i) {
        const float x = s.xyz[3*i], y = s.xyz[3*i+1], z = s.xyz[3*i+2];
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        PointT p;
        p.x = x; p.y = y; p.z = z;
        p.intensity = hi ? s.intensity[i] : 0.f;
        c->push_back(p);
    }
    return c;
}
}  // namespace

Relocalizer::Relocalizer() : impl_(new Impl) {}
Relocalizer::~Relocalizer() = default;

void Relocalizer::set_config(const RelocConfig& c) { cfg_ = c; }

bool Relocalizer::load_prior(const std::string& pcd_path) {
    Cloud::Ptr raw(new Cloud);
    if (pcl::io::loadPCDFile<PointT>(pcd_path, *raw) < 0) {
        std::cerr << "[Relocalizer] cannot load prior map: " << pcd_path << "\n";
        return false;
    }
    voxel(raw, impl_->prior, cfg_.map_voxel_m);
    if (cfg_.gi_floor_filter) filter_floor(impl_->prior, impl_->prior_score, cfg_.gi_floor_band_m, cfg_.gi_wall_top_m);
    else                      *impl_->prior_score = *impl_->prior;
    if (!impl_->prior_score->empty()) {
        impl_->prior_kdtree.setInputCloud(impl_->prior_score);   // NN over the wall band, for the score
        impl_->kdtree_ready = true;
    }
    std::cout << "[Relocalizer] prior map " << pcd_path << ": " << raw->size()
              << " -> " << impl_->prior->size() << " pts (voxel " << cfg_.map_voxel_m << " m); "
              << "scoring walls=" << impl_->prior_score->size()
              << (cfg_.gi_floor_filter ? " (floor filtered)" : " (no floor filter)") << "\n";
    return !impl_->prior->empty();
}

void Relocalizer::add_scan(const LioCloud& scan) {
    impl_->scans.push_back(to_pcl(scan));
    while (int(impl_->scans.size()) > cfg_.submap_scans) impl_->scans.pop_front();
    impl_->submap_dirty = true;
}

std::size_t Relocalizer::submap_size() const { return impl_->submap->size(); }
std::size_t Relocalizer::prior_size()  const { return impl_->prior->size(); }

std::vector<float> Relocalizer::prior_xyz() const {
    std::vector<float> out;
    out.reserve(impl_->prior->size() * 3);
    for (const auto& p : *impl_->prior) { out.push_back(p.x); out.push_back(p.y); out.push_back(p.z); }
    return out;
}

std::vector<float> Relocalizer::submap_xyz(const Eigen::Matrix4f& T) const {
    std::vector<float> out;
    out.reserve(impl_->submap->size() * 3);
    for (const auto& p : *impl_->submap) {
        out.push_back(T(0,0)*p.x + T(0,1)*p.y + T(0,2)*p.z + T(0,3));
        out.push_back(T(1,0)*p.x + T(1,1)*p.y + T(1,2)*p.z + T(1,3));
        out.push_back(T(2,0)*p.x + T(2,1)*p.y + T(2,2)*p.z + T(2,3));
    }
    return out;
}

void Relocalizer::seed(const Eigen::Matrix4f& T) {
    T_map_odom_ = T;
    seed_yaw_ = std::atan2(T(1, 0), T(0, 0));   // reference heading for the yaw gate
    seeded_ = true;
}

void Relocalizer::ensure_submap() {
    if (!impl_->submap_dirty) return;
    Cloud::Ptr merged(new Cloud);
    for (const auto& s : impl_->scans) *merged += *s;
    if (merged->empty()) { impl_->submap->clear(); return; }
    voxel(merged, impl_->submap, cfg_.submap_voxel_m);
    impl_->submap_dirty = false;
}

bool Relocalizer::align(double* fitness_out) {
    if (fitness_out) *fitness_out = -1.0;
    if (!seeded_ || impl_->prior->empty()) return false;

    ensure_submap();
    if (impl_->submap->empty()) return false;

    auto& g = impl_->gicp;
    g.setMaximumIterations(cfg_.gicp_max_iter);
    g.setMaxCorrespondenceDistance(cfg_.gicp_max_corr_m);
    g.setInputSource(impl_->submap);                 // odom
    g.setInputTarget(impl_->prior);                  // map
    Cloud aligned;
    g.align(aligned, T_map_odom_);                   // init guess = current estimate
    if (!g.hasConverged()) return false;

    const double fit = g.getFitnessScore();
    if (fitness_out) *fitness_out = fit;
    const Eigen::Matrix4f T = g.getFinalTransformation();
    const double jump = (T.block<3,1>(0,3) - T_map_odom_.block<3,1>(0,3)).norm();
    if (fit > cfg_.fitness_max || jump > cfg_.max_jump_m) return false;   // reject: keep last

    // Yaw gate: reject a solution rotated far from the (fixed) seed heading — kills 180deg flips
    // in symmetric scenes. Assumes a roughly-correct seed yaw (constant boot heading).
    if (cfg_.max_yaw_deg > 0.0) {
        double dyaw = std::atan2(double(T(1,0)), double(T(0,0))) - double(seed_yaw_);
        while (dyaw >  M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
        if (std::abs(dyaw) > cfg_.max_yaw_deg * M_PI / 180.0) return false;
    }

    T_map_odom_ = T;
    return true;
}

bool Relocalizer::gicp_measure(const Eigen::Matrix4f& init, Eigen::Matrix4f* T_out, double* fitness_out) {
    if (fitness_out) *fitness_out = -1.0;
    if (impl_->prior->empty()) return false;
    ensure_submap();
    if (impl_->submap->empty()) return false;

    auto& g = impl_->gicp;
    g.setMaximumIterations(cfg_.gicp_max_iter);
    g.setMaxCorrespondenceDistance(cfg_.gicp_max_corr_m);
    g.setInputSource(impl_->submap);                 // odom
    g.setInputTarget(impl_->prior);                  // map
    Cloud aligned;
    g.align(aligned, init);                          // init guess = EKF-predicted T_map_odom
    if (!g.hasConverged()) return false;
    if (fitness_out) *fitness_out = g.getFitnessScore();
    if (T_out) *T_out = g.getFinalTransformation();
    return true;
}

float Relocalizer::score_candidate(const Eigen::Matrix4f& T, int stride, float* median_out) const {
    if (median_out) *median_out = -1.f;
    if (!impl_->kdtree_ready || impl_->submap_score->empty()) return 0.f;
    if (stride < 1) stride = 1;

    const float thr    = cfg_.gi_inlier_thresh_m;
    const float thr_sq = thr * thr;                       // FLANN returns squared distances
    std::vector<int>   idx(1);
    std::vector<float> d2(1);
    std::vector<float> inlier_d;                          // inlier distances, for the median
    std::size_t considered = 0, inliers = 0;

    for (std::size_t i = 0; i < impl_->submap_score->size(); i += std::size_t(stride)) {
        const PointT& p = (*impl_->submap_score)[i];
        PointT q;                                         // submap point mapped into the map frame
        q.x = T(0,0)*p.x + T(0,1)*p.y + T(0,2)*p.z + T(0,3);
        q.y = T(1,0)*p.x + T(1,1)*p.y + T(1,2)*p.z + T(1,3);
        q.z = T(2,0)*p.x + T(2,1)*p.y + T(2,2)*p.z + T(2,3);
        ++considered;
        if (impl_->prior_kdtree.nearestKSearch(q, 1, idx, d2) > 0 && d2[0] <= thr_sq) {
            ++inliers;
            inlier_d.push_back(std::sqrt(d2[0]));
        }
    }
    if (considered == 0) return 0.f;
    if (median_out && !inlier_d.empty()) {
        auto mid = inlier_d.begin() + inlier_d.size() / 2;
        std::nth_element(inlier_d.begin(), mid, inlier_d.end());
        *median_out = *mid;
    }
    return float(inliers) / float(considered);
}

namespace {
// map<-odom candidate for a base heading yaw. UWB pins the base xy (tag offset rotated out per
// yaw); base z comes from odom; odom is then chained off the base.  T_map_odom = T_map_base * T_odom_base^-1
Eigen::Matrix4f candidate_T(float yaw, const Eigen::Vector2f& tag_xy_map,
                            const Eigen::Vector3f& tag_in_pelvis, float z_base,
                            const Eigen::Matrix4f& T_odom_base_inv) {
    const Eigen::Matrix3f Rz =
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    const Eigen::Vector3f off = Rz * tag_in_pelvis;       // tag offset expressed in map
    Eigen::Matrix4f T_map_base = Eigen::Matrix4f::Identity();
    T_map_base.block<3,3>(0,0) = Rz;
    T_map_base(0,3) = tag_xy_map.x() - off.x();           // pelvis xy = tag xy - rotated offset xy
    T_map_base(1,3) = tag_xy_map.y() - off.y();
    T_map_base(2,3) = z_base;                             // z from odom (UWB gives no z)
    return T_map_base * T_odom_base_inv;
}
float ang_dist_deg(float a, float b) {                    // smallest |a-b| on the yaw circle (deg)
    float d = std::fmod(std::fabs(a - b), 360.f);
    return d > 180.f ? 360.f - d : d;
}
}  // namespace

GlobalInitResult Relocalizer::global_init(const Eigen::Vector2f& tag_xy_map,
                                          const Eigen::Vector3f& tag_in_pelvis,
                                          const Eigen::Matrix4f& T_odom_base,
                                          bool verbose) {
    GlobalInitResult r;
    if (!impl_->kdtree_ready) { std::cerr << "[Relocalizer] global_init: no prior\n"; return r; }
    ensure_submap();
    if (impl_->submap->empty()) { std::cerr << "[Relocalizer] global_init: empty submap\n"; return r; }
    if (cfg_.gi_floor_filter) filter_floor(impl_->submap, impl_->submap_score, cfg_.gi_floor_band_m, cfg_.gi_wall_top_m);
    else                      *impl_->submap_score = *impl_->submap;
    if (impl_->submap_score->empty()) { std::cerr << "[Relocalizer] global_init: no wall points in submap\n"; return r; }

    const float z_base = T_odom_base(2,3);
    const Eigen::Matrix4f T_odom_base_inv = T_odom_base.inverse();
    auto T_at = [&](float yaw_deg) {
        return candidate_T(yaw_deg * float(M_PI) / 180.f, tag_xy_map, tag_in_pelvis, z_base, T_odom_base_inv);
    };

    std::printf("[global_init] scoring: submap walls=%zu, prior walls=%zu (inlier<%.2fm)\n",
                impl_->submap_score->size(), impl_->prior_score->size(), cfg_.gi_inlier_thresh_m);

    // Stage 1: coarse sweep 0..360 (subsampled), robust inlier-ratio score.
    const float step = cfg_.gi_yaw_step_deg > 0.1f ? cfg_.gi_yaw_step_deg : 10.f;
    std::vector<std::pair<float,float>> curve;            // (yaw_deg, inlier_ratio)
    for (float yaw = 0.f; yaw < 360.f - 1e-3f; yaw += step)
        curve.emplace_back(yaw, score_candidate(T_at(yaw), cfg_.gi_coarse_stride, nullptr));

    if (verbose) {                                        // print the whole yaw landscape
        std::printf("[global_init] yaw curve (deg:ratio):");
        for (std::size_t i = 0; i < curve.size(); ++i) {
            if (i % 12 == 0) std::printf("\n  ");
            std::printf("%3.0f:%.2f ", curve[i].first, curve[i].second);
        }
        std::printf("\n");
    }

    // best coarse peak + best well-separated competitor (180deg symmetry margin).
    std::size_t best_i = 0;
    for (std::size_t i = 1; i < curve.size(); ++i)
        if (curve[i].second > curve[best_i].second) best_i = i;
    const float best_yaw = curve[best_i].first, best_r = curve[best_i].second;
    float second_r = 0.f, second_yaw = -1.f;
    for (const auto& c : curve)
        if (ang_dist_deg(c.first, best_yaw) >= cfg_.gi_peak_sep_deg && c.second > second_r) {
            second_r = c.second; second_yaw = c.first;
        }
    const float confidence = best_r > 1e-6f ? (best_r - second_r) / best_r : 0.f;

    std::printf("[global_init] coarse: best yaw=%.0f r=%.2f | 2nd yaw=%.0f r=%.2f | confidence=%.2f\n",
                best_yaw, best_r, second_yaw, second_r, confidence);

    // Stage 2/3: refine yaw around the coarse peak (full submap, no subsample).
    auto refine = [&](float center, float half_range, float fine_step) {
        float bo = center, bs = -1.f;
        for (float d = -half_range; d <= half_range + 1e-3f; d += fine_step) {
            float yaw = std::fmod(center + d + 360.f, 360.f);
            float s = score_candidate(T_at(yaw), 1, nullptr);
            if (s > bs) { bs = s; bo = yaw; }
        }
        return std::make_pair(bo, bs);
    };
    auto [yaw2, r2] = refine(best_yaw, step,        step / 5.f);   // e.g. +/-10 step 2
    (void)r2;
    auto [yaw3, r3] = refine(yaw2,     step / 5.f,  step / 20.f);  // e.g. +/-2  step 0.5

    r.yaw_deg      = yaw3;
    r.inlier_ratio = r3;
    r.confidence   = confidence;
    r.T_map_odom   = T_at(yaw3);

    // Temporal-consistency: track the recent best yaws; a weakly-constrained spot's correct yaw stays
    // stable frame-to-frame even when its single-frame confidence is low.
    recent_best_yaws_.push_back(yaw3);
    while (int(recent_best_yaws_.size()) > cfg_.gi_stable_frames) recent_best_yaws_.pop_front();
    bool stable = int(recent_best_yaws_.size()) >= cfg_.gi_stable_frames;
    if (stable) {
        const float newest = recent_best_yaws_.back();
        for (float y : recent_best_yaws_)
            if (ang_dist_deg(y, newest) > cfg_.gi_stable_yaw_tol_deg) { stable = false; break; }
    }
    const bool inlier_ok = (r3 >= cfg_.gi_min_inlier_ratio);
    const bool conf_ok   = (confidence >= cfg_.gi_min_confidence);
    r.ok = inlier_ok && (conf_ok || stable);

    std::printf("[global_init] refine yaw=%.1f r=%.2f conf=%.2f stable=%d/%d -> %s%s\n",
                r.yaw_deg, r.inlier_ratio, confidence, int(recent_best_yaws_.size()), cfg_.gi_stable_frames,
                r.ok ? "ACCEPT" : "reject", (r.ok && !conf_ok && stable) ? " (by stability)" : "");

    if (r.ok) { T_map_odom_ = r.T_map_odom; seed_yaw_ = yaw3 * float(M_PI) / 180.f; seeded_ = true; }
    return r;
}

float Relocalizer::score_yaw(const Eigen::Vector2f& tag_xy_map, const Eigen::Vector3f& tag_in_pelvis,
                             const Eigen::Matrix4f& T_odom_base, float yaw_deg,
                             Eigen::Matrix4f* T_out, float* median_out) {
    if (!impl_->kdtree_ready) return 0.f;
    ensure_submap();
    if (impl_->submap->empty()) return 0.f;
    if (cfg_.gi_floor_filter) filter_floor(impl_->submap, impl_->submap_score, cfg_.gi_floor_band_m, cfg_.gi_wall_top_m);
    else                      *impl_->submap_score = *impl_->submap;
    const Eigen::Matrix4f T = candidate_T(yaw_deg * float(M_PI) / 180.f, tag_xy_map, tag_in_pelvis,
                                          T_odom_base(2,3), T_odom_base.inverse());
    if (T_out) *T_out = T;
    return score_candidate(T, 1, median_out);
}

bool Relocalizer::dump_overlay(const std::string& pcd_path) const {
    return dump_overlay(pcd_path, T_map_odom_);
}

bool Relocalizer::dump_overlay(const std::string& pcd_path, const Eigen::Matrix4f& T) const {
    if (impl_->prior->empty() && impl_->submap->empty()) return false;   // nothing to write
    Cloud out;
    out.reserve(impl_->prior->size() + impl_->submap->size());
    for (PointT p : *impl_->prior) { p.intensity = 0.f;   out.push_back(p); }   // prior: low
    Cloud tf;
    pcl::transformPointCloud(*impl_->submap, tf, T);
    for (PointT p : tf)            { p.intensity = 200.f; out.push_back(p); }   // submap: high
    if (out.empty()) return false;
    out.width = out.size(); out.height = 1; out.is_dense = false;
    return pcl::io::savePCDFileBinary(pcd_path, out) == 0;
}

}  // namespace kist
