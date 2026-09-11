#pragma once

// ObstacleGridPublisher — bridges the ROS-free ObstacleGrid to rviz2 over DDS. It
// publishes the grid as nav_msgs/OccupancyGrid (rt/obstacle_grid) and the robot base
// as geometry_msgs/PoseStamped (rt/robot_pose), both in the LIO odom frame
// ("camera_init"), so rviz shows the occupancy map with a heading arrow next to the
// registered cloud. Publish-only, for visualization/validation — the planner consumes
// the ObstacleGrid struct directly, not this wire form.

#include "mapping/obstacle_grid.hpp"
#include "mapping/costmap.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace unitree::robot {
template <typename T> class ChannelPublisher;
}
namespace nav_msgs::msg::dds_ {
class OccupancyGrid_;
}
namespace geometry_msgs::msg::dds_ {
class PoseStamped_;
}
namespace sensor_msgs::msg::dds_ {
class PointCloud2_;
}

namespace kist {

class ObstacleGridPublisher {
public:
    ObstacleGridPublisher();          // out-of-line (unique_ptr to incomplete ChannelPublisher)
    ~ObstacleGridPublisher();

    bool start(int domain_id, const std::string& network_interface = "",
               const std::string& grid_topic      = "rt/obstacle_grid",
               const std::string& pose_topic      = "rt/robot_pose",
               const std::string& costmap_topic   = "rt/costmap",
               const std::string& path_topic      = "rt/plan",
               const std::string& raw_path_topic  = "rt/plan_raw",
               const std::string& clearance_topic = "rt/clearance",
               const std::string& medial_topic    = "rt/medial_axis",
               const std::string& frame_id        = "camera_init");   // LIO odom frame

    // Convert + publish one grid snapshot (and the robot pose from grid.robot_*).
    void publish(const ObstacleGrid& g, const ObstacleGridConfig& cfg);

    // Publish the costmap as an OccupancyGrid (cost 0..lethal -> 0..100) on rt/costmap.
    void publish_costmap(const Costmap& cm);

    // Publish a planned path as a PointCloud2 on rt/plan — the waypoints densified to
    // point_spacing_m so it reads as a continuous line in rviz (no Path/Marker IDL here).
    // Empty waypoints publishes an empty cloud (clears the last path).
    void publish_path(const std::vector<std::pair<float, float>>& waypoints,
                      float point_spacing_m = 0.05f);

    // The raw A* grid path (pre-smoothing) on rt/plan_raw — for comparison with the smoothed
    // bubble path. Already grid-dense, so published as-is (no extra densify).
    void publish_path_raw(const std::vector<std::pair<float, float>>& waypoints);

    // Debug/insight views of the geometry the centreline planning is built on:
    //   clearance field -> OccupancyGrid (rt/clearance): distance-to-wall heatmap, bright in
    //     the middle of corridors (the "drivable envelope"); max_m maps to full brightness.
    //   medial axis -> PointCloud2 (rt/medial_axis): the ridge = corridor centreline points.
    void publish_clearance(const Costmap& cm, const std::vector<float>& clr, float max_m = 2.0f);
    void publish_medial(const std::vector<std::pair<float, float>>& points);

    ObstacleGridPublisher(const ObstacleGridPublisher&) = delete;
    ObstacleGridPublisher& operator=(const ObstacleGridPublisher&) = delete;

private:
    std::string frame_;
    using GridPub = unitree::robot::ChannelPublisher<nav_msgs::msg::dds_::OccupancyGrid_>;
    using PosePub = unitree::robot::ChannelPublisher<geometry_msgs::msg::dds_::PoseStamped_>;
    using PathPub = unitree::robot::ChannelPublisher<sensor_msgs::msg::dds_::PointCloud2_>;
    std::unique_ptr<GridPub> grid_pub_;
    std::unique_ptr<PosePub> pose_pub_;
    std::unique_ptr<GridPub> costmap_pub_;
    std::unique_ptr<PathPub> path_pub_;
    std::unique_ptr<PathPub> raw_path_pub_;
    std::unique_ptr<GridPub> clearance_pub_;
    std::unique_ptr<PathPub> medial_pub_;
};

} // namespace kist
