#include "localization/relocalizer.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>

#include <cmath>
#include <deque>
#include <iostream>

namespace kist {

using PointT = pcl::PointXYZI;
using Cloud  = pcl::PointCloud<PointT>;

struct Relocalizer::Impl {
    Cloud::Ptr             prior{new Cloud};   // downsampled prior map (map frame)
    std::deque<Cloud::Ptr> scans;              // recent registered scans (odom frame)
    Cloud::Ptr             submap{new Cloud};  // merged + downsampled submap (odom frame)
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
    std::cout << "[Relocalizer] prior map " << pcd_path << ": " << raw->size()
              << " -> " << impl_->prior->size() << " pts (voxel " << cfg_.map_voxel_m << " m)\n";
    return !impl_->prior->empty();
}

void Relocalizer::add_scan(const LioCloud& scan) {
    impl_->scans.push_back(to_pcl(scan));
    while (int(impl_->scans.size()) > cfg_.submap_scans) impl_->scans.pop_front();
    impl_->submap_dirty = true;
}

std::size_t Relocalizer::submap_size() const { return impl_->submap->size(); }
std::size_t Relocalizer::prior_size()  const { return impl_->prior->size(); }

void Relocalizer::seed(const Eigen::Matrix4f& T) { T_map_odom_ = T; seeded_ = true; }

bool Relocalizer::align(double* fitness_out) {
    if (fitness_out) *fitness_out = -1.0;
    if (!seeded_ || impl_->prior->empty()) return false;

    if (impl_->submap_dirty) {                       // rebuild merged + downsampled submap
        Cloud::Ptr merged(new Cloud);
        for (const auto& s : impl_->scans) *merged += *s;
        if (merged->empty()) return false;
        voxel(merged, impl_->submap, cfg_.submap_voxel_m);
        impl_->submap_dirty = false;
    }
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

    T_map_odom_ = T;
    return true;
}

bool Relocalizer::dump_overlay(const std::string& pcd_path) const {
    Cloud out;
    out.reserve(impl_->prior->size() + impl_->submap->size());
    for (PointT p : *impl_->prior) { p.intensity = 0.f;   out.push_back(p); }   // prior: low
    Cloud tf;
    pcl::transformPointCloud(*impl_->submap, tf, T_map_odom_);
    for (PointT p : tf)            { p.intensity = 200.f; out.push_back(p); }   // submap: high
    if (out.empty()) return false;
    out.width = out.size(); out.height = 1; out.is_dense = false;
    return pcl::io::savePCDFileBinary(pcd_path, out) == 0;
}

}  // namespace kist
