#pragma once

// ObstacleGridPublisher — bridges the ROS-free ObstacleGrid to rviz2 over DDS. It
// publishes the grid as nav_msgs/OccupancyGrid (rt/obstacle_grid) and the robot base
// as geometry_msgs/PoseStamped (rt/robot_pose), both in the LIO odom frame
// ("camera_init"), so rviz shows the occupancy map with a heading arrow next to the
// registered cloud. Publish-only, for visualization/validation — the planner consumes
// the ObstacleGrid struct directly, not this wire form.

#include "mapping/obstacle_grid.hpp"

#include <memory>
#include <string>

namespace unitree::robot {
template <typename T> class ChannelPublisher;
}
namespace nav_msgs::msg::dds_ {
class OccupancyGrid_;
}
namespace geometry_msgs::msg::dds_ {
class PoseStamped_;
}

namespace kist {

class ObstacleGridPublisher {
public:
    ObstacleGridPublisher();          // out-of-line (unique_ptr to incomplete ChannelPublisher)
    ~ObstacleGridPublisher();

    bool start(int domain_id, const std::string& network_interface = "",
               const std::string& grid_topic = "rt/obstacle_grid",
               const std::string& pose_topic = "rt/robot_pose",
               const std::string& frame_id   = "camera_init");   // LIO odom frame

    // Convert + publish one grid snapshot (and the robot pose from grid.robot_*).
    void publish(const ObstacleGrid& g, const ObstacleGridConfig& cfg);

    ObstacleGridPublisher(const ObstacleGridPublisher&) = delete;
    ObstacleGridPublisher& operator=(const ObstacleGridPublisher&) = delete;

private:
    std::string frame_;
    using GridPub = unitree::robot::ChannelPublisher<nav_msgs::msg::dds_::OccupancyGrid_>;
    using PosePub = unitree::robot::ChannelPublisher<geometry_msgs::msg::dds_::PoseStamped_>;
    std::unique_ptr<GridPub> grid_pub_;
    std::unique_ptr<PosePub> pose_pub_;
};

} // namespace kist
