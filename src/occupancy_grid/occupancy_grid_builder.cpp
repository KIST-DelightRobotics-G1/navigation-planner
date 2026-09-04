#include "occupancy_grid/occupancy_grid_builder.hpp"

#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace kist {

bool OccupancyGridBuilder::start(DataBuffer<UnitreePointCloud>& lidar_src,
                                 DataBuffer<LabeledCloud>&      cloud_src,
                                 DataBuffer<CalibratedPose>&    pose_src,
                                 const GridConfig& cfg) {
    if (running_) return true;
    lidar_src_ = &lidar_src;
    cloud_src_ = &cloud_src;
    pose_src_  = &pose_src;
    cfg_       = cfg;
    running_ = true;
    thread_  = std::thread(&OccupancyGridBuilder::run, this);
    return true;
}

void OccupancyGridBuilder::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void OccupancyGridBuilder::run() {
    pthread_setname_np(pthread_self(), "occ-grid");
    int64_t last_lidar = -1, last_cloud = -1;
    bool    world_anchored = false;

    while (running_) {
        auto lidar = lidar_src_->GetData();
        auto cloud = cloud_src_->GetData();
        auto pose  = pose_src_->GetData();
        const bool new_lidar = lidar && lidar->stamp_ns != last_lidar && lidar->point_count() > 0;
        const bool new_cloud = cloud && cloud->stamp_ns != last_cloud && !cloud->empty();
        if (!new_lidar && !new_cloud) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (new_lidar) last_lidar = lidar->stamp_ns;
        if (new_cloud) last_cloud = cloud->stamp_ns;

        Pose2D p;
        if (pose) {
            // Trusted pose -> world-anchored ACCUMULATION. The grid persists, so
            // integrate only the newly-arrived cloud each cycle; the other
            // sensor's past evidence is still in the belief.
            p = { pose->x, pose->y, pose->yaw };
            if (!world_anchored) { grid_reset(grid_, cfg_, p.x, p.y); world_anchored = true; }
            else { grid_.robot_x = p.x; grid_.robot_y = p.y; }
            grid_.robot_yaw = p.yaw;
            grid_decay(grid_, cfg_);
            if (new_lidar) grid_integrate_lidar(grid_, *lidar, p, cfg_);
            if (new_cloud) grid_integrate_labeled(grid_, *cloud, p, cfg_);
        } else {
            // No pose -> robot-centered MEMORYLESS snapshot. The grid is wiped
            // each cycle, so integrate BOTH latest clouds every time (else the
            // display flickers between LiDAR-only and camera-only frames).
            world_anchored = false;
            p = { 0.f, 0.f, 0.f };
            grid_reset(grid_, cfg_, 0.f, 0.f);
            if (lidar && lidar->point_count() > 0) grid_integrate_lidar(grid_, *lidar, p, cfg_);
            if (cloud && !cloud->empty())          grid_integrate_labeled(grid_, *cloud, p, cfg_);
        }

        // Current-frame instance-per-cell map (per-frame indices; used only to
        // split touching same-class objects this cycle — never accumulated).
        cv::Mat inst_map;
        if (cloud && !cloud->instance.empty()) {
            inst_map = cv::Mat(grid_.n, grid_.n, CV_32S, cv::Scalar(-1));
            const float cs = std::cos(p.yaw), sn = std::sin(p.yaw);
            const size_t n = cloud->size();
            for (size_t i = 0; i < n; ++i) {
                const float rx = cloud->xyz[3*i], ry = cloud->xyz[3*i+1], rz = cloud->xyz[3*i+2];
                if (rz < cfg_.min_z || rz > cfg_.max_z) continue;
                const float wx = p.x + cs*rx - sn*ry, wy = p.y + sn*rx + cs*ry;
                int ix, iy;
                if (grid_.world_to_cell(wx, wy, ix, iy))
                    inst_map.at<int>(iy, ix) = cloud->instance[i];
            }
        }

        grid_.stamp_ns = std::max(last_lidar, last_cloud);
        grid_buf_.SetData(grid_);
        objects_buf_.SetData(extract_clusters(grid_, cfg_, inst_map));  // cluster inline (+ instance split)
        processed_.fetch_add(1, std::memory_order_relaxed);
    }
}

} // namespace kist
