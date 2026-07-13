// Live LiDAR stream check (robot LAN needed):
//   ./test_unitree_pointcloud_reader <network_interface> [domain_id] [topic]
// Prints frame rate, point count, and a sample point once per second.

#include "unitree/unitree_pointcloud_reader.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: %s <network_interface> [domain_id] [topic]\n", argv[0]);
        return 1;
    }
    const std::string iface  = argv[1];
    const int         domain = argc >= 3 ? std::atoi(argv[2]) : 0;

    std::signal(SIGINT, [](int) { g_stop = true; });

    auto& reader = UnitreePointcloudReader::instance();
    bool ok = argc >= 4 ? reader.start(domain, iface, argv[3])
                        : reader.start(domain, iface);
    if (!ok)
        return 1;

    int64_t last_stamp = -1;
    int     frames     = 0;
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        auto c = reader.cloud_buf.GetDataWithTime();
        if (!c.HasData()) {
            std::printf("cloud: <empty> (no frames / stale)\n");
            frames = 0;
            continue;
        }
        if (c.data->stamp_ns != last_stamp) {
            last_stamp = c.data->stamp_ns;
            ++frames;
        }
        std::printf("frame_id=%s  points=%zu  age=%.0fms  first=(%.2f %.2f %.2f)\n",
                    c.data->frame_id.c_str(), c.data->point_count(), c.GetAgeMs(),
                    c.data->empty() ? 0.0 : c.data->xyz[0],
                    c.data->empty() ? 0.0 : c.data->xyz[1],
                    c.data->empty() ? 0.0 : c.data->xyz[2]);
    }

    reader.stop();
    return 0;
}
