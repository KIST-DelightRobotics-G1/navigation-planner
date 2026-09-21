#pragma once

// DDS Rx for cortex subtask commands (rt/cortex/nav/cmd, SubtaskCmd). nav handles action=="move_to"
// only; args[0] is a destination name resolved against the catalog (config/destinations.yaml) into a
// Goal on result(). Unsupported action / bad args -> Goal{valid=false, disp=Failed}; cancel ->
// Goal{valid=false, disp=Cancelled}. Every emitted Goal carries the subtask (plan_id/index/action)
// so the controller can report it back on rt/cortex/nav/state.
//
// Separate from DestinationPublisher on purpose: the publisher advertises the catalog outward, this
// consumes subtask commands inward. A consumer reads result() and drives the planner + follower.

#include "common/data_buffer.hpp"
#include "goal_generation/goal.hpp"

#include <kist_nav.hpp>   // generated: kist_msgs::SubtaskCmd

#include <map>
#include <string>

namespace unitree::robot { template <typename T> class ChannelSubscriber; }

namespace kist {

inline constexpr const char* kNavCmdTopic = "rt/cortex/nav/cmd";   // ROS2 /cortex/nav/cmd

class GoalCommandReceiver {
public:
    GoalCommandReceiver() = default;
    ~GoalCommandReceiver();
    GoalCommandReceiver(const GoalCommandReceiver&) = delete;
    GoalCommandReceiver& operator=(const GoalCommandReceiver&) = delete;

    // Load the catalog (for name -> pose resolution) and subscribe. False on load / DDS failure.
    bool start(int domain_id, const std::string& network_interface,
               const std::string& destinations_yaml,
               const std::string& topic = kNavCmdTopic);
    void stop();

    DataBuffer<Goal>& result() { return goal_buf_; }   // resolved goal (or valid=false + disposition)
    size_t catalog_size() const { return catalog_.size(); }

private:
    void on_command(const void* message);
    bool load(const std::string& yaml_path);

    using Sub = unitree::robot::ChannelSubscriber<kist_msgs::SubtaskCmd>;
    std::shared_ptr<Sub>        sub_;
    std::map<std::string, Goal> catalog_;   // name -> Goal (yaw in rad, valid=true)
    DataBuffer<Goal>            goal_buf_;
};

} // namespace kist
