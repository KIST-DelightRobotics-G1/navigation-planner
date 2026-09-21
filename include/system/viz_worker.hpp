// VizWorker — the visualization thread (~10 Hz): read the grid / costmap / path buffers and
// publish them to rviz (occupancy + costmap + clearance + medial axis + paths). Publish-only,
// no control path. Owns its thread + loop; deps injected at start(). Holds a copy of the grid
// config (static) for the resolution/band metadata the publisher needs.
#pragma once

#include "common/data_buffer.hpp"
#include "lio/lio_receiver.hpp"
#include "lio/lio_transform_producer.hpp"
#include "localization/map_odom.hpp"
#include "mapping/obstacle_grid_publisher.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"
#include "route_planner/perception/obstacle_grid_builder/obstacle_grid.hpp"
#include "route_planner/planner/astar_planner/path.hpp"

#include <atomic>
#include <thread>

namespace kist {

class VizWorker {
public:
    ~VizWorker() { stop(); }

    void start(DataBuffer<ObstacleGrid>& grid_buf, DataBuffer<Costmap>& costmap_buf,
               DataBuffer<Path>& path_buf, ObstacleGridPublisher& pub, ObstacleGridConfig gcfg,
               LioTransformProducer& prod, LioReceiver& rx, DataBuffer<MapOdom>& mapodom,
               bool survey, std::vector<float> prior_map_xyz);
    void stop();

private:
    void run();

    ObstacleGridConfig        gcfg_;          // copy (static): resolution + height band for publish
    DataBuffer<ObstacleGrid>* grid_buf_    = nullptr;
    DataBuffer<Costmap>*      costmap_buf_ = nullptr;
    DataBuffer<Path>*         path_buf_    = nullptr;
    ObstacleGridPublisher*    pub_         = nullptr;
    LioTransformProducer*     prod_        = nullptr;   // lidar/pelvis pose + sway
    LioReceiver*              rx_          = nullptr;   // registered scan (for the sway cloud)
    DataBuffer<MapOdom>*      mapodom_     = nullptr;   // map->odom (for the survey map pose)
    bool                      survey_      = false;     // navigation.survey: print robot map pose
    std::vector<float>        prior_map_xyz_;           // prior map (map frame) for the odom overlay

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
