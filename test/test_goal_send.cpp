// Publishes one goal command (NavGoalCommand) so the receiver can be tested
// without a peer. Name from argv (empty = cancel/stop).
//   ./test_goal_send <destination_name> [config_path]
//   ./test_goal_send                     -> cancel

#include "goal_generation/goal_command_receiver.hpp"   // kNavGoalTopic + kist_nav.hpp
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string name = (argc >= 2) ? argv[1] : "";                 // "" = cancel
    const std::string cfg  = (argc >= 3) ? argv[2] : "config/config.yaml";
    kist::Config::instance().load(cfg);
    const auto& root = kist::Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;

    unitree::robot::ChannelFactory::Instance()->Init(domain, "");
    using Pub = unitree::robot::ChannelPublisher<kist_msgs::NavGoalCommand>;
    Pub pub(kist::kNavGoalTopic);
    pub.InitChannel();

    kist_msgs::NavGoalCommand msg;
    msg.name(name);
    for (int i = 0; i < 5; ++i) {   // repeat a few times to beat discovery/QoS timing
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        msg.seq(i);
        msg.stamp_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
        pub.Write(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("sent goal command name='%s' on %s\n", name.c_str(), kist::kNavGoalTopic);
    return 0;
}
