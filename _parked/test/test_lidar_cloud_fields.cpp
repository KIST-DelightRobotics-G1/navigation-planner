// LiDAR cloud field inspector. Deskew needs a PER-POINT timestamp; the utlidar
// PointCloud2 relay may or may not carry one. This prints the PointCloud2 field
// layout (name / offset / datatype / count) + point_step + point count, so we can
// see whether a per-point time channel (t / timestamp / offset_time) exists.
//   ./test_lidar_cloud_fields [config.yaml] [topic]
// default topic: rt/utlidar/cloud_livox_mid360

#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};
static std::atomic<bool> g_printed{false};

static const char* dtype_name(int d) {
    switch (d) {
        case 1: return "INT8";   case 2: return "UINT8";
        case 3: return "INT16";  case 4: return "UINT16";
        case 5: return "INT32";  case 6: return "UINT32";
        case 7: return "FLOAT32";case 8: return "FLOAT64";
        default: return "?";
    }
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string cfg   = (argc >= 2) ? argv[1] : "config/config.yaml";
    const std::string topic = (argc >= 3) ? argv[2] : "rt/utlidar/cloud_livox_mid360";

    kist::Config::instance().load(cfg);
    const auto& root = kist::Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!kist::apply_dds_config(root)) return 1;

    std::signal(SIGINT, [](int) { g_stop = true; });

    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain, "");
        static unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_> sub(topic);
        sub.InitChannel([](const void* m) {
            if (g_printed.exchange(true)) return;              // print the first frame only
            const auto& c = *static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(m);
            const int64_t t = int64_t(c.header().stamp().sec()) * 1000000000LL +
                              c.header().stamp().nanosec();
            const auto npts = (c.point_step() ? c.data().size() / c.point_step() : 0);
            std::printf("frame=%s stamp=%lld  width=%u height=%u point_step=%u  points=%zu\n",
                        c.header().frame_id().c_str(), static_cast<long long>(t),
                        c.width(), c.height(), c.point_step(), static_cast<size_t>(npts));
            std::printf("fields (%zu):\n", static_cast<size_t>(c.fields().size()));
            uint32_t time_off = 0; bool has_time = false;
            for (const auto& f : c.fields()) {
                std::printf("  %-14s offset=%-3u datatype=%-7s count=%u\n",
                            f.name().c_str(), f.offset(), dtype_name(f.datatype()), f.count());
                if (f.name() == "time" && f.datatype() == 7) { time_off = f.offset(); has_time = true; }
            }
            // Confirm the time channel's semantics: relative seconds (~0..0.1 for a
            // 10 Hz frame) vs absolute. LIO needs to know which.
            if (has_time && c.point_step() && npts) {
                const uint8_t* blob = c.data().data();
                float tmin = 1e30f, tmax = -1e30f, t0 = 0;
                for (size_t i = 0; i < npts; ++i) {
                    float v; std::memcpy(&v, blob + i * c.point_step() + time_off, sizeof(v));
                    if (i == 0) t0 = v;
                    if (v < tmin) tmin = v;
                    if (v > tmax) tmax = v;
                }
                std::printf("time channel: first=%.6f min=%.6f max=%.6f span=%.6f\n",
                            t0, tmin, tmax, tmax - tmin);
                std::printf("  -> span ~0.1 => RELATIVE seconds within the frame; "
                            "huge => absolute\n");
            }
        }, 1);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "DDS init failed: %s\n", e.what());
        return 1;
    }

    std::printf("waiting for one PointCloud2 on \"%s\" (domain %d)...\n", topic.c_str(), domain);
    int waited = 0;
    while (!g_stop && !g_printed.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (++waited % 4 == 0) std::printf("...nothing yet (%ds)\n", waited / 2);
    }
    return 0;
}
