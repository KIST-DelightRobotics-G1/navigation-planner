#pragma once

// GoalReceiver — subscribes the navigation goal rviz's "2D Goal Pose" tool publishes
// (geometry_msgs/PoseStamped on rt/goal_pose) and hands out the latest goal in the odom
// frame. ROS-free (unitree_sdk2 / CycloneDDS). Click a goal in rviz -> the planner replans.

#include "common/data_buffer.hpp"
#include "goal_generation/goal.hpp"   // the unified kist::Goal (x/y/yaw/dock/valid/name)

#include <memory>
#include <string>

namespace unitree::robot {
template <typename T> class ChannelSubscriber;
}
namespace geometry_msgs::msg::dds_ {
class PoseStamped_;
}

namespace kist {

// GoalReceiver produces the same kist::Goal as the mission layer, but for ad-hoc rviz clicks:
// a raw point with dock OFF (has_yaw=true from the clicked orientation, but dock.align=false,
// so the follower just drives there and stops). Named destinations (with dock) come from the
// mission layer's GoalCommandReceiver instead.

class GoalReceiver {
public:
    GoalReceiver();
    ~GoalReceiver();

    bool start(int domain_id, const std::string& network_interface = "",
               const std::string& topic = "rt/goal_pose");
    void stop();

    DataBuffer<Goal> goal_buf;   // latest clicked goal

    GoalReceiver(const GoalReceiver&) = delete;
    GoalReceiver& operator=(const GoalReceiver&) = delete;

private:
    void on_goal(const void* message);

    using GoalSub = unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PoseStamped_>;
    std::unique_ptr<GoalSub> sub_;
};

} // namespace kist
