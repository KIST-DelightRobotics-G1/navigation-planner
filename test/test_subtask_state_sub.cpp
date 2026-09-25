// Subscribes to rt/cortex/nav/state and prints the nav module's SubtaskState — confirms the
// wire contract and lets you watch the run-to-completion loop (RUNNING -> ... -> DONE / FAILED).
//   ./test_subtask_state_sub [config_path]   (run the planner + send a SubtaskCmd)

#include "controller/subtask_state_publisher.hpp"   // kNavStateTopic
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <kist_nav.hpp>                              // cortex_msgs::msg::dds_::SubtaskState_

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};

static const char* status_name(uint8_t s) {
    switch (s) {
        case 0: return "IDLE";
        case 1: return "RUNNING";
        case 2: return "DONE";
        case 3: return "FAILED";
        default: return "?";
    }
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string cfg = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(cfg);
    const auto& root = kist::Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;

    std::signal(SIGINT, [](int) { g_stop = true; });
    unitree::robot::ChannelFactory::Instance()->Init(domain, "");

    uint8_t last = 255;   // 10 Hz stream -> only print on a status transition
    using Sub = unitree::robot::ChannelSubscriber<cortex_msgs::msg::dds_::SubtaskState_>;
    Sub sub(kist::kNavStateTopic);
    sub.InitChannel([&last](const void* m) {
        const auto& msg = *static_cast<const cortex_msgs::msg::dds_::SubtaskState_*>(m);
        const uint8_t s = msg.status();
        if (s == last) return;                       // suppress the steady 10 Hz repeats
        std::printf("[nav-state] %-7s plan='%s' idx=%u action='%s' prog=%.2f note='%s'\n",
                    status_name(s), msg.plan_id().c_str(), msg.index(), msg.action().c_str(),
                    msg.progress(), msg.detail().c_str());
        last = s;
    }, 10);

    std::printf("listening on %s (Ctrl+C to quit)\n", kist::kNavStateTopic);
    while (!g_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return 0;
}
