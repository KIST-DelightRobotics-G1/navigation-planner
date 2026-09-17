#pragma once

// ObstacleGridPublisher — bridges the ROS-free ObstacleGrid to rviz2 over DDS. It
// publishes the grid as nav_msgs/OccupancyGrid (rt/obstacle_grid) and the robot base
// as geometry_msgs/PoseStamped (rt/robot_pose), both in the LIO odom frame
// ("camera_init"), so rviz shows the occupancy map with a heading arrow next to the
// registered cloud. Publish-only, for visualization/validation — the planner consumes
// the ObstacleGrid struct directly, not this wire form.

#include "lio/robot_transforms.hpp"
#include "route_planner/perception/obstacle_grid_builder/obstacle_grid.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"

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
               const std::string& lidar_pose_topic  = "rt/lidar_pose",
               const std::string& pelvis_pose_topic = "rt/pelvis_pose",
               const std::string& sway_cloud_topic  = "rt/cloud_sway",
               const std::string& prior_map_topic   = "rt/prior_map",
               const std::string& frame_id        = "camera_init");   // LIO odom frame

    // Convert + publish one grid snapshot (and the robot pose from grid.robot_*).
    void publish(const ObstacleGrid& g, const ObstacleGridConfig& cfg);

    // Full-3D poses for the sway before/after comparison: T_odom_lidar (rt/lidar_pose) wobbles
    // with the head/torso sway; T_odom_pelvis (rt/pelvis_pose) is the waist-FK-stabilized base.
    // View both as rviz Pose displays while walking — lidar tilts/bobs, pelvis stays level.
    void publish_frames(const RobotTransforms& tf);

    // A "shaking" cloud on rt/cloud_sway: the registered scan re-rotated about the robot base by
    // the torso sway (lidar-vs-pelvis rotation), so in odom it visibly swings with the gait — the
    // sway magnitude the pelvis stabilization removes. `xyz` is flat odom points (x0,y0,z0,...).
    void publish_cloud(const std::vector<float>& xyz);

    // The prior map drawn IN THE ODOM FRAME (caller pre-transforms it by T_odom_map) on
    // rt/prior_map, so it overlays the live obstacle grid — the global frame's structure vs the
    // live scene. When localised it sits on the live walls; misalignment shows the map->odom error.
    void publish_prior(const std::vector<float>& xyz);

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
    std::unique_ptr<PosePub> lidar_pose_pub_;    // T_odom_lidar (raw, sways)
    std::unique_ptr<PosePub> pelvis_pose_pub_;   // T_odom_pelvis (stabilized)
    std::unique_ptr<PathPub> sway_cloud_pub_;    // registered scan + sway (shaking, rt/cloud_sway)
    std::unique_ptr<PathPub> prior_map_pub_;     // prior map in odom (global-frame overlay)
};

} // namespace kist
