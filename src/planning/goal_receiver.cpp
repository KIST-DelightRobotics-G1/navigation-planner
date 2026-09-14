#include "planning/goal_receiver.hpp"

#include <unitree/idl/ros2/PoseStamped_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <cmath>
#include <iostream>

namespace kist {

GoalReceiver::GoalReceiver() = default;
GoalReceiver::~GoalReceiver() { stop(); }

bool GoalReceiver::start(int domain_id, const std::string& network_interface,
                         const std::string& topic) {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);  // no-op if inited
        sub_.reset(new GoalSub(topic));
        sub_->InitChannel([this](const void* m) { on_goal(m); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[GoalReceiver] DDS init failed: " << e.what() << "\n";
        return false;
    }
    std::cout << "[GoalReceiver] listening on " << topic << " (rviz 2D Goal Pose)\n";
    return true;
}

void GoalReceiver::stop() { sub_.reset(); }

void GoalReceiver::on_goal(const void* message) {
    const auto& msg = *static_cast<const geometry_msgs::msg::dds_::PoseStamped_*>(message);
    const auto& p = msg.pose().position();
    const auto& q = msg.pose().orientation();

    Goal g;
    g.x = float(p.x());
    g.y = float(p.y());
    g.yaw = float(std::atan2(2.0 * (q.w()*q.z() + q.x()*q.y()),
                             1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z())));
    g.has_yaw = true;         // rviz supplies an orientation ...
    g.valid   = true;
    g.in_map  = false;        // rviz clicks are in the displayed odom frame -> no conversion
    g.name    = "";           // ad-hoc, not a catalog destination
    // ... but dock stays default (align/approach OFF) -> just drive there and stop.
    goal_buf.SetData(g);
    std::cout << "[GoalReceiver] goal (" << g.x << ", " << g.y << ") yaw "
              << g.yaw * 57.2958f << " deg\n";
}

} // namespace kist
