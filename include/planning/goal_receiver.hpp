#pragma once

// GoalReceiver — subscribes the navigation goal rviz's "2D Goal Pose" tool publishes
// (geometry_msgs/PoseStamped on rt/goal_pose) and hands out the latest goal in the odom
// frame. ROS-free (unitree_sdk2 / CycloneDDS). Click a goal in rviz -> the planner replans.

#include "common/data_buffer.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace unitree::robot {
template <typename T> class ChannelSubscriber;
}
namespace geometry_msgs::msg::dds_ {
class PoseStamped_;
}

namespace kist {

struct Goal {
    int64_t stamp_ns = 0;
    float   x = 0.f, y = 0.f;   // odom
    float   yaw = 0.f;          // desired finish heading (rad)
};

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
