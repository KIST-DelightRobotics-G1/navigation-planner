#include "controller/subtask_state_publisher.hpp"

#include <kist_nav.hpp>   // generated: kist_msgs::SubtaskState / Header / Time
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
                                    float progress, const std::string& note) {
    if (!pub_) return;
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    kist_msgs::Time t;
    t.sec(static_cast<int32_t>(ns / 1000000000LL));
    t.nanosec(static_cast<uint32_t>(ns % 1000000000LL));
    kist_msgs::Header h;
    h.stamp(t);
    h.frame_id("");

    kist_msgs::SubtaskState msg;
    msg.header(h);
    msg.plan_id(plan_id);
    msg.index(index);
    msg.action(action);
    msg.status(static_cast<uint8_t>(status));
    msg.progress(progress);
    msg.note(note);
    pub_->Write(msg);
}

void SubtaskStatePublisher::stop() {
    pub_.reset();
}

} // namespace kist
