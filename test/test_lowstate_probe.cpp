// Waist-joint probe: prints the G1 waist angles at ~5 Hz so we can see whether
// they actually move as the robot walks. If yaw/roll/pitch swing during gait,
// forward kinematics off these will capture the torso (sensor) bob the pelvis
// odom misses; if they stay flat, the bob is elsewhere (pelvis) and FK won't help.
//
//   ./test_lowstate_probe [config_path]

#include "unitree/unitree_state_reader.hpp"
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

    auto& rd = kist::UnitreeStateReader::instance();
    if (!rd.start(domain, "")) return 1;
    std::printf("waist joints (deg) — stand still vs walk, watch the swing (Ctrl+C quit)\n");

    float ymin=1e9f,ymax=-1e9f,rmin=1e9f,rmax=-1e9f,pmin=1e9f,pmax=-1e9f;
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto s = rd.state_buf.GetData();
        if (!s) { std::printf("no lowstate yet\n"); continue; }
        // consumer parses the waist joints out of the full state: 12=yaw 13=roll 14=pitch
        const float y = float(s->motors[12].q)*57.2958f;
        const float r = float(s->motors[13].q)*57.2958f;
        const float p = float(s->motors[14].q)*57.2958f;
        ymin=std::min(ymin,y); ymax=std::max(ymax,y);
        rmin=std::min(rmin,r); rmax=std::max(rmax,r);
        pmin=std::min(pmin,p); pmax=std::max(pmax,p);
        std::printf("waist yaw=%6.1f roll=%6.1f pitch=%6.1f | swing y=%.1f r=%.1f p=%.1f deg\n",
                    y, r, p, ymax-ymin, rmax-rmin, pmax-pmin);
    }
    rd.stop();
    return 0;
}
