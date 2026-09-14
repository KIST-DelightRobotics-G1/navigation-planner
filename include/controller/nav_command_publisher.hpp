#pragma once

// NavCommandPublisher — publishes a NavCommand as a geometry_msgs/Twist on rt/kist/nav/cmd_vel,
// the contract kist-gearsonic-inference consumes to drive the G1. Publishing this is what
// actually MOVES the robot. Kept behind the controller worker's arming gate + safety.

#include "controller/nav_command.hpp"

#include <memory>
#include <string>

namespace unitree::robot { template <typename T> class ChannelPublisher; }
namespace geometry_msgs::msg::dds_ { class Twist_; }

namespace kist {

class NavCommandPublisher {
public:
    NavCommandPublisher();          // out-of-line (unique_ptr to incomplete ChannelPublisher)
    ~NavCommandPublisher();

    bool start(int domain_id, const std::string& network_interface = "",
               const std::string& topic = "rt/kist/nav/cmd_vel");
    void publish(const NavCommand& c);   // vx->linear.x, vy->linear.y, vyaw->angular.z
    void stop();                         // publishes a final zero (stop) and closes

    NavCommandPublisher(const NavCommandPublisher&) = delete;
    NavCommandPublisher& operator=(const NavCommandPublisher&) = delete;

private:
    using TwistPub = unitree::robot::ChannelPublisher<geometry_msgs::msg::dds_::Twist_>;
    std::unique_ptr<TwistPub> pub_;
};

} // namespace kist
