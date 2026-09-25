#include "controller/subtask_state_publisher.hpp"

#include <kist_nav.hpp>   // generated: cortex_msgs::msg::dds_::SubtaskState_
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <chrono>
#include <iostream>

namespace kist {

SubtaskStatePublisher::SubtaskStatePublisher() = default;      // ChannelPublisher complete here
SubtaskStatePublisher::~SubtaskStatePublisher() { stop(); }

bool SubtaskStatePublisher::start(int domain_id, const std::string& network_interface,
                                  const std::string& topic) {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);  // no-op if inited
        pub_.reset(new Pub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[SubtaskStatePublisher] DDS init failed: " << e.what() << "\n";
        return false;
    }
    std::cout << "[SubtaskStatePublisher] state=" << topic << "\n";
    return true;
}

void SubtaskStatePublisher::publish(const std::string& plan_id, uint16_t index,
                                    const std::string& action, SubtaskStatus status,
                                    float progress, const std::string& detail) {
    if (!pub_) return;
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    cortex_msgs::msg::dds_::SubtaskState_ msg;
    msg.stamp_ns(ns);                      // was a std_msgs-style Header; cortex and VLA use ns
    msg.plan_id(plan_id);
    msg.index(index);
    msg.action(action);
    msg.status(static_cast<uint8_t>(status));
    msg.progress(progress);
    msg.detail(detail);                    // wire field is `detail`; internally still `note`
    pub_->Write(msg);
}

void SubtaskStatePublisher::stop() {
    pub_.reset();
}

} // namespace kist
