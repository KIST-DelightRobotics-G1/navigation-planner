// Waist-joint probe: prints the G1 waist angles at ~5 Hz so we can see whether
// they actually move as the robot walks. If yaw/roll/pitch swing during gait,
// forward kinematics off these will capture the torso (sensor) bob the pelvis
// odom misses; if they stay flat, the bob is elsewhere (pelvis) and FK won't help.
//
//   ./test_lowstate_probe [config_path]

#include "unitree/g1_lowstate_reader.hpp"
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
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

    auto& rd = kist::G1LowStateReader::instance();
    if (!rd.start(domain, "")) return 1;
    std::printf("waist joints (deg) — stand still vs walk, watch the swing (Ctrl+C quit)\n");

    float ymin=1e9f,ymax=-1e9f,rmin=1e9f,rmax=-1e9f,pmin=1e9f,pmax=-1e9f;
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto w = rd.waist_buf.GetData();
        if (!w) { std::printf("no lowstate yet\n"); continue; }
        const float y = w->yaw*57.2958f, r = w->roll*57.2958f, p = w->pitch*57.2958f;
        ymin=std::min(ymin,y); ymax=std::max(ymax,y);
        rmin=std::min(rmin,r); rmax=std::max(rmax,r);
        pmin=std::min(pmin,p); pmax=std::max(pmax,p);
        std::printf("waist yaw=%6.1f roll=%6.1f pitch=%6.1f | swing y=%.1f r=%.1f p=%.1f deg\n",
                    y, r, p, ymax-ymin, rmax-rmin, pmax-pmin);
    }
    rd.stop();
    return 0;
}
