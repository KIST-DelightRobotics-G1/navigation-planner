// Subscribes to rt/kist/nav/status and prints the controller's navigation state — confirms the
// NavStatus wire contract and lets you watch the run-to-completion loop (DRIVING -> ... -> ARRIVED).
//   ./test_nav_status_sub [config_path]   (run the planner + send a goal with test_goal_send)

#include "controller/nav_status_publisher.hpp"   // kNavStatusTopic
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <kist_nav.hpp>                           // kist_msgs::NavStatus

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};

static const char* state_name(uint8_t s) {
    switch (s) {
        case 0: return "IDLE";
        case 1: return "DRIVING";
        case 2: return "ALIGNING";
        case 3: return "APPROACHING";
        case 4: return "ARRIVED";
        case 5: return "BLOCKED";
        case 6: return "NO_ROUTE";
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

    uint8_t last = 255;   // print every sample, but flag the state transitions
    using Sub = unitree::robot::ChannelSubscriber<kist_msgs::NavStatus>;
    Sub sub(kist::kNavStatusTopic);
    sub.InitChannel([&last](const void* m) {
        const auto& msg = *static_cast<const kist_msgs::NavStatus*>(m);
        const uint8_t s = msg.state();
        const char* edge = (s != last) ? "  <-- transition" : "";
        std::printf("[nav-status] seq %llu  state=%-11s name='%s'%s\n",
                    (unsigned long long)msg.seq(), state_name(s), msg.name().c_str(), edge);
        last = s;
    }, 1);

    std::printf("listening on %s (Ctrl+C to quit)\n", kist::kNavStatusTopic);
    while (!g_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return 0;
}
