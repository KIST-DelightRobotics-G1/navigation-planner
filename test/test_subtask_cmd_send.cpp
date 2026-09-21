// Publishes one SubtaskCmd on rt/cortex/nav/cmd so the nav module can be tested without the
// orchestrator. Sends a move_to to a destination, or a cancel.
//   ./test_subtask_cmd_send <destination>          -> move_to [<destination>]
//   ./test_subtask_cmd_send --cancel               -> cancel (plan/index below)
//   ./test_subtask_cmd_send <dest> [plan_id] [index] [config_path]

#include "goal_generation/goal_command_receiver.hpp"   // kNavCmdTopic + kist_nav.hpp
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string arg1    = (argc >= 2) ? argv[1] : "";
    const std::string plan_id = (argc >= 3) ? argv[2] : "p-test-0001";
    const uint16_t    index   = (argc >= 4) ? uint16_t(std::stoi(argv[3])) : 0;
    const std::string cfg     = (argc >= 5) ? argv[4] : "config/config.yaml";
    const bool        cancel  = (arg1 == "--cancel");

    kist::Config::instance().load(cfg);
    const auto& root = kist::Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;

    unitree::robot::ChannelFactory::Instance()->Init(domain, "");
    using Pub = unitree::robot::ChannelPublisher<kist_msgs::SubtaskCmd>;
    Pub pub(kist::kNavCmdTopic);
    pub.InitChannel();

    kist_msgs::SubtaskCmd msg;
    msg.plan_id(plan_id);
    msg.index(index);
    msg.action(cancel ? "" : "move_to");
    msg.args(cancel ? std::vector<std::string>{} : std::vector<std::string>{arg1});
    msg.instruction("");
    msg.cancel(cancel);

    for (int i = 0; i < 5; ++i) {   // repeat a few times to beat discovery/QoS timing
        pub.Write(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (cancel) std::printf("sent cancel (plan=%s idx=%u) on %s\n", plan_id.c_str(), index, kist::kNavCmdTopic);
    else        std::printf("sent move_to '%s' (plan=%s idx=%u) on %s\n",
                            arg1.c_str(), plan_id.c_str(), index, kist::kNavCmdTopic);
    return 0;
}
