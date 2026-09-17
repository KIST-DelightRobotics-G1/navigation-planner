#include "system/viz_worker.hpp"

#include "route_planner/perception/costmap_builder/clearance.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <utility>
#include <vector>

namespace kist {

namespace {
double yaw_of(const Eigen::Quaterniond& q) {
    return std::atan2(2 * (q.w()*q.z() + q.x()*q.y()), 1 - 2 * (q.y()*q.y() + q.z()*q.z()));
}
}  // namespace

void VizWorker::start(DataBuffer<ObstacleGrid>& grid_buf, DataBuffer<Costmap>& costmap_buf,
                      DataBuffer<Path>& path_buf, ObstacleGridPublisher& pub, ObstacleGridConfig gcfg,
                      LioTransformProducer& prod, LioReceiver& rx, DataBuffer<MapOdom>& mapodom,
                      std::vector<float> prior_map_xyz) {
    grid_buf_ = &grid_buf; costmap_buf_ = &costmap_buf; path_buf_ = &path_buf; pub_ = &pub;
    gcfg_ = gcfg; prod_ = &prod; rx_ = &rx; mapodom_ = &mapodom;
    prior_map_xyz_ = std::move(prior_map_xyz);
    survey_ = [] { const char* v = std::getenv("NAV_SURVEY"); return v && v[0] == '1'; }();
    running_ = true;
    thread_ = std::thread(&VizWorker::run, this);
}

void VizWorker::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void VizWorker::run() {
    auto last_survey = std::chrono::steady_clock::now();
    auto last_prior  = std::chrono::steady_clock::now();
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
        // Survey (NAV_SURVEY=1): print the robot's MAP-frame pose so you can drive it to a spot
        // (e.g. in front of the fridge) and read the x/y/yaw to paste into destinations.yaml.
        if (survey_) {
            const auto now = std::chrono::steady_clock::now();
            if (now - last_survey >= std::chrono::seconds(1)) {
                last_survey = now;
                auto mo = mapodom_->GetData();
                auto rt = prod_->out_buf.GetData();
                if (mo && rt) {
                    const Eigen::Matrix4f& T = mo->T_map_odom;   // p_map = T * p_odom
                    const double px = rt->T_odom_pelvis.translation.x();
                    const double py = rt->T_odom_pelvis.translation.y();
                    const double yo = yaw_of(rt->T_odom_pelvis.rotation);
                    const double mx = T(0,0)*px + T(0,1)*py + T(0,3);
                    const double my = T(1,0)*px + T(1,1)*py + T(1,3);
                    const double ym = yo + std::atan2(T(1,0), T(0,0));
                    std::printf("[survey] robot MAP pose: x=%.3f y=%.3f yaw_deg=%.1f  "
                                "-> destinations.yaml (x,y; yaw_deg=arrival heading)\n",
                                mx, my, ym * 57.2958);
                } else {
                    std::printf("[survey] waiting for map->odom lock...\n");
                }
            }
        }
        // Global-frame overlay (~2 Hz): the prior map drawn IN ODOM (transform by T_odom_map),
        // so rt/prior_map sits on the live obstacle grid — see the map->odom alignment in rviz.
        if (!prior_map_xyz_.empty()) {
            const auto now = std::chrono::steady_clock::now();
            if (now - last_prior >= std::chrono::milliseconds(500)) {
                last_prior = now;
                if (auto mo = mapodom_->GetData()) {
                    const Eigen::Matrix4f T = mo->T_map_odom.inverse();   // T_odom_map
                    const std::size_t n = prior_map_xyz_.size() / 3;
                    std::vector<float> out(n * 3);
                    for (std::size_t i = 0; i < n; ++i) {
                        const Eigen::Vector4f p(prior_map_xyz_[3*i], prior_map_xyz_[3*i+1],
                                                prior_map_xyz_[3*i+2], 1.f);
                        const Eigen::Vector4f q = T * p;
                        out[3*i] = q.x(); out[3*i+1] = q.y(); out[3*i+2] = q.z();
                    }
                    pub_->publish_prior(out);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));   // ~10 Hz to rviz
    }
}

} // namespace kist
