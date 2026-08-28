// Live UWB fix check (robot LAN + UWB transmitter running):
//   ./test_uwb_reader [config_path]
// Interface/domain come from config/config.yaml. Subscribes to the UWB pose
// topic (rt/kist/uwb/pose) and prints receive rate + latest position once per
// second. Ctrl-C to stop.
//
// The UwbReceiver assembly is EMBEDDED from the sibling kist-ext-sensor-io
// checkout (not vendored here) — this runner is the navigation-side connection
// point for global-position fixes, and doubles as the reference for how the
// future orchestrator pulls fix() in-process. Both repos share an identical
// common/ (data_buffer, config), so linking ext's uwb_receiver composes cleanly.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "system/uwb_receiver.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <csignal>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);

    const auto unitree_cfg = Config::instance().root()["unitree"];
    const auto domain_id   = unitree_cfg["domain_id"].as<int>();
    if (!apply_dds_config(Config::instance().root())) return 1;  // NIC + tuning from config/cyclonedds.xml

    UwbReceiver rx;
    if (!rx.start(domain_id, ""))
        return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    std::printf("[test_uwb_reader] subscribing on domain=%d\n", domain_id);

    int     frames = 0;
    int64_t last_stamp = -1;
    auto    window_start = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto fix = rx.fix().GetData();
        if (fix && fix->stamp_ns != last_stamp) {
            last_stamp = fix->stamp_ns;
            ++frames;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window_start >= std::chrono::seconds(1)) {
            window_start = now;
            if (fix)
                std::printf("rate=%3dHz  latest xyz = (%.2f, %.2f, %.2f)\n",
                            frames, fix->x, fix->y, fix->z);
            else
                std::printf("no fix (buffer empty)\n");
            frames = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    rx.stop();
    return 0;
}
