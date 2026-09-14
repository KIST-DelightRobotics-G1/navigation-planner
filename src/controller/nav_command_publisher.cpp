#include "controller/nav_command_publisher.hpp"

#include <unitree/idl/ros2/Twist_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <iostream>

namespace kist {

NavCommandPublisher::NavCommandPublisher() = default;    // ChannelPublisher complete here
NavCommandPublisher::~NavCommandPublisher() { stop(); }

bool NavCommandPublisher::start(int domain_id, const std::string& network_interface,
                         const std::string& topic) {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);  // no-op if inited
        pub_.reset(new TwistPub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[NavCommandPublisher] DDS init failed: " << e.what() << "\n";
        return false;
    }
    std::cout << "[NavCommandPublisher] cmd_vel=" << topic << "\n";
    return true;
}

void NavCommandPublisher::publish(const NavCommand& c) {
    if (!pub_) return;
    geometry_msgs::msg::dds_::Twist_ msg;
    msg.linear().x(c.vx);  msg.linear().y(c.vy);  msg.linear().z(0.0);
    msg.angular().x(0.0);  msg.angular().y(0.0);  msg.angular().z(c.vyaw);
    pub_->Write(msg);
}

void NavCommandPublisher::stop() {
    if (pub_) publish(NavCommand{});   // final stop on the wire
    pub_.reset();
}

} // namespace kist
