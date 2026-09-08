// Runs GoalCommandReceiver: subscribes to rt/kist/nav/goal, resolves each named
// command against config/destinations.yaml, and logs the resolved Goal. Pair with
// test_goal_send to verify the goal-command contract.
//   ./test_goal_recv [config_path]

#include "goal_generation/goal_command_receiver.hpp"
#include "common/config.hpp"
#include "common/dds_config.hpp"

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

    std::string dfile = "config/destinations.yaml";
    if (const auto nav = root["navigation"])
        dfile = nav["destinations_file"].as<std::string>(dfile);

    std::signal(SIGINT, [](int) { g_stop = true; });

    kist::GoalCommandReceiver rx;
    if (!rx.start(domain, "", dfile)) return 1;
    std::printf("waiting for goal commands on %s (send with test_goal_send)\n", kist::kNavGoalTopic);

    // Also report the resolved Goal buffer (yaw in deg) on change.
    int64_t last_seen = -1;
    while (!g_stop) {
        if (auto g = rx.result().GetData()) {
            const int64_t tag = int64_t(g->valid) * 1000 + int64_t(g->x * 100);
            if (tag != last_seen) {
                if (g->valid)
                    std::printf("  -> Goal '%s' (%.2f, %.2f) yaw %.0f deg\n",
                                g->name.c_str(), g->x, g->y, g->yaw * 180.0f / 3.14159265f);
                else
                    std::printf("  -> no active goal (cancel)\n");
                last_seen = tag;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    rx.stop();
    return 0;
}
