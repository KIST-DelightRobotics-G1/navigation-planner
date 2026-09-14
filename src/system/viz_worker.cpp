#include "system/viz_worker.hpp"

#include "route_planner/perception/costmap_builder/clearance.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <cstddef>
#include <vector>

namespace kist {

void VizWorker::start(DataBuffer<ObstacleGrid>& grid_buf, DataBuffer<Costmap>& costmap_buf,
                      DataBuffer<Path>& path_buf, ObstacleGridPublisher& pub, ObstacleGridConfig gcfg,
                      LioTransformProducer& prod, LioReceiver& rx) {
    grid_buf_ = &grid_buf; costmap_buf_ = &costmap_buf; path_buf_ = &path_buf; pub_ = &pub;
    gcfg_ = gcfg; prod_ = &prod; rx_ = &rx;
    running_ = true;
    thread_ = std::thread(&VizWorker::run, this);
}

void VizWorker::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void VizWorker::run() {
    while (running_) {
        if (auto grid = grid_buf_->GetData()) pub_->publish(*grid, gcfg_);
        if (auto cm = costmap_buf_->GetData(); cm && !cm->empty()) {
            pub_->publish_costmap(*cm);
            const auto clr = clearance_field(*cm);
            pub_->publish_clearance(*cm, clr);
            pub_->publish_medial(medial_axis(*cm, clr));
        }
        if (auto path = path_buf_->GetData()) {
            pub_->publish_path(path->waypoints, gcfg_.resolution_m);
            pub_->publish_path_raw(path->raw_waypoints);
        }
        if (auto rt = prod_->out_buf.GetData()) {
            pub_->publish_frames(*rt);            // lidar (sways) vs pelvis (stable) poses
            // Shaking cloud: re-rotate the registered scan about the robot base by the torso sway
            // (lidar-vs-pelvis rotation). In odom it swings by exactly the sway the pelvis removes.
            if (auto scan = rx_->cloud_buf.GetData(); scan && !scan->xyz.empty()) {
                const Eigen::Quaterniond R = rt->T_odom_lidar.rotation *
                                             rt->T_odom_pelvis.rotation.conjugate();
                const Eigen::Vector3d b = rt->T_odom_pelvis.translation;
                const std::size_t n = scan->xyz.size() / 3;
                std::vector<float> out(n * 3);
                for (std::size_t i = 0; i < n; ++i) {
                    const Eigen::Vector3d p(scan->xyz[3*i], scan->xyz[3*i+1], scan->xyz[3*i+2]);
                    const Eigen::Vector3d q = R * (p - b) + b;   // p' = R_sway*(p - base) + base
                    out[3*i] = float(q.x()); out[3*i+1] = float(q.y()); out[3*i+2] = float(q.z());
                }
                pub_->publish_cloud(out);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));   // ~10 Hz to rviz
    }
}

} // namespace kist
