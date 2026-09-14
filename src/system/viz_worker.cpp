#include "system/viz_worker.hpp"

#include "route_planner/perception/costmap_builder/clearance.hpp"

#include <chrono>

namespace kist {

void VizWorker::start(DataBuffer<ObstacleGrid>& grid_buf, DataBuffer<Costmap>& costmap_buf,
                      DataBuffer<Path>& path_buf, ObstacleGridPublisher& pub, ObstacleGridConfig gcfg) {
    grid_buf_ = &grid_buf; costmap_buf_ = &costmap_buf; path_buf_ = &path_buf; pub_ = &pub;
    gcfg_ = gcfg;
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
        std::this_thread::sleep_for(std::chrono::milliseconds(100));   // ~10 Hz to rviz
    }
}

} // namespace kist
