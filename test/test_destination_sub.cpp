// Subscribes to rt/kist/nav/destinations and prints the catalog — confirms the
// NavDestinationList wire contract decodes on the receiving side.
//   ./test_destination_sub [config_path]   (pair with test_destination_publisher)

#include "goal_generation/destination_publisher.hpp"   // kNavDestinationsTopic + kist_nav.hpp
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string cfg = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(cfg);
    const auto& root = kist::Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;

    std::signal(SIGINT, [](int) { g_stop = true; });
    unitree::robot::ChannelFactory::Instance()->Init(domain, "");

    using Sub = unitree::robot::ChannelSubscriber<kist_msgs::NavDestinationList>;
    Sub sub(kist::kNavDestinationsTopic);
    sub.InitChannel([](const void* m) {
        const auto& msg = *static_cast<const kist_msgs::NavDestinationList*>(m);
        std::printf("[dest-sub] seq %llu — %zu destinations:\n",
                    (unsigned long long)msg.seq(), msg.destinations().size());
        for (const auto& d : msg.destinations())
            std::printf("    %-14s (%.2f, %.2f) yaw %.0f deg\n",
                        d.name().c_str(), d.x(), d.y(), d.yaw_deg());
    }, 1);

    std::printf("listening on %s (Ctrl+C to quit)\n", kist::kNavDestinationsTopic);
    while (!g_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return 0;
}
