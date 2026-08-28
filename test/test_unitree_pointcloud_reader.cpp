// Live LiDAR stream check (robot LAN needed):
//   ./test_unitree_pointcloud_reader [config_path]
// Everything (interface, domain, topic, processing) comes from
// config/config.yaml. Prints frame age, point count, and a sample point
// once per second. Ctrl-C to stop.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "pointcloud/pointcloud_processor.hpp"
#include "unitree/unitree_pointcloud_reader.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);
    const auto& root = Config::instance().root();

    const auto unitree_cfg = root["unitree"];
    const auto domain_id   = unitree_cfg["domain_id"].as<int>();
    if (!apply_dds_config(Config::instance().root())) return 1;  // NIC + tuning from config/cyclonedds.xml

    std::signal(SIGINT, [](int) { g_stop = true; });

    // assembly: processing stage plugged into the reader's receive thread
    auto& reader = UnitreePointCloudReader::instance();
    reader.set_processor(
        [proc = PointCloudProcessor{
             pointcloud_processor_options_from_yaml(root["pointcloud_processor"])}](
            UnitreePointCloud& f) { proc.process(f); });

    if (!reader.start(domain_id, ""))
        return 1;

    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        auto cloud = reader.cloud_buf.GetDataWithTime();
        if (!cloud.HasData()) {
            std::printf("no cloud (buffer empty)\n");
            continue;
        }

        const auto& f = *cloud.data;
        const std::size_t n = f.point_count();
        std::printf("age=%6.1fms  points=%6zu  frame_id=%s",
                    cloud.GetAgeMs(), n, f.frame_id.c_str());
        if (n > 0) {
            // frame extents — with the processor enabled these must sit
            // inside the configured filter ranges (z band, min radius)
            float z_min = f.xyz[2], z_max = f.xyz[2];
            float r2_min = f.xyz[0] * f.xyz[0] + f.xyz[1] * f.xyz[1];
            for (std::size_t i = 0; i < f.xyz.size(); i += 3) {
                const float z  = f.xyz[i + 2];
                const float r2 = f.xyz[i] * f.xyz[i] + f.xyz[i + 1] * f.xyz[i + 1];
                if (z < z_min) z_min = z;
                if (z > z_max) z_max = z;
                if (r2 < r2_min) r2_min = r2;
            }
            const std::size_t mid = (n / 2) * 3;
            std::printf("  z=[%.2f, %.2f]  r_min=%.2f  sample=(%.2f, %.2f, %.2f)",
                        z_min, z_max, std::sqrt(r2_min),
                        f.xyz[mid], f.xyz[mid + 1], f.xyz[mid + 2]);
        }
        std::printf("\n");
    }

    reader.stop();
    return 0;
}
