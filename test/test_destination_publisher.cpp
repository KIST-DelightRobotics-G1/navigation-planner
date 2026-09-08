// Publishes the destination catalog (config/destinations.yaml) on
// rt/kist/nav/destinations so a peer can list the addressable goals.
//   ./test_destination_publisher [config_path]

#include "goal_generation/destination_publisher.hpp"
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

    kist::DestinationPublisher pub;
    if (!pub.start(domain, "", dfile)) return 1;
    std::printf("publishing %zu destinations on %s (Ctrl+C to quit)\n",
                pub.count(), kist::kNavDestinationsTopic);

    while (!g_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    pub.stop();
    return 0;
}
