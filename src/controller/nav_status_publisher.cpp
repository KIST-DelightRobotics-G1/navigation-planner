#include "controller/nav_status_publisher.hpp"

#include <kist_nav.hpp>   // generated: kist_msgs::NavStatus
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <chrono>
#include <iostream>

namespace kist {

NavStatusPublisher::NavStatusPublisher() = default;      // ChannelPublisher complete here
NavStatusPublisher::~NavStatusPublisher() { stop(); }

bool NavStatusPublisher::start(int domain_id, const std::string& network_interface,
                               const std::string& topic) {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);  // no-op if inited
        pub_.reset(new StatusPub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[NavStatusPublisher] DDS init failed: " << e.what() << "\n";
        return false;
    }
    std::cout << "[NavStatusPublisher] status=" << topic << "\n";
    return true;
}

void NavStatusPublisher::publish(const std::string& name, NavState state) {
    if (!pub_) return;
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    kist_msgs::NavStatus msg;
    msg.seq(seq_++);
    msg.stamp_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
    msg.name(name);
    msg.state(static_cast<uint8_t>(state));
    pub_->Write(msg);
}

void NavStatusPublisher::stop() {
    pub_.reset();
}

} // namespace kist
