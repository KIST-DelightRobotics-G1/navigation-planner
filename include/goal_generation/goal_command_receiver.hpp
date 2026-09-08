#pragma once

// DDS Rx for goal commands (rt/kist/nav/goal, NavGoalCommand). A received name is
// resolved against the destination catalog (config/destinations.yaml) into a Goal
// on result(); an empty or unknown name yields Goal{valid=false} (cancel/stop).
//
// Separate from DestinationPublisher on purpose: the publisher advertises the
// catalog outward, this consumes goal commands inward. A consumer reads result()
// and drives the planner (set_goal) + follower (set_goal_yaw).

#include "common/data_buffer.hpp"
#include "goal_generation/goal.hpp"

#include <kist_nav.hpp>   // generated: kist_msgs::NavGoalCommand

#include <map>
#include <string>

namespace unitree::robot { template <typename T> class ChannelSubscriber; }

namespace kist {

inline constexpr const char* kNavGoalTopic = "rt/kist/nav/goal";

class GoalCommandReceiver {
public:
    GoalCommandReceiver() = default;
    ~GoalCommandReceiver();
    GoalCommandReceiver(const GoalCommandReceiver&) = delete;
    GoalCommandReceiver& operator=(const GoalCommandReceiver&) = delete;

    // Load the catalog (for name -> pose resolution) and subscribe. False on
    // load / DDS failure.
    bool start(int domain_id, const std::string& network_interface,
               const std::string& destinations_yaml,
               const std::string& topic = kNavGoalTopic);
    void stop();

    DataBuffer<Goal>& result() { return goal_buf_; }   // resolved goal (or valid=false)
    size_t catalog_size() const { return catalog_.size(); }

private:
    void on_command(const void* message);
    bool load(const std::string& yaml_path);

    using Sub = unitree::robot::ChannelSubscriber<kist_msgs::NavGoalCommand>;
    std::shared_ptr<Sub>        sub_;
    std::map<std::string, Goal> catalog_;   // name -> Goal (yaw in rad, valid=true)
    DataBuffer<Goal>            goal_buf_;
};

} // namespace kist
