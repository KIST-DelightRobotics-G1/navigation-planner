#include "system/nav_system.hpp"

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

namespace kist {

namespace { NavSystem* g_self = nullptr; }

bool NavSystem::start(const std::string& config_path) {
    Config::instance().load(config_path);
    const auto& root = Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!apply_dds_config(root)) return false;

    // ── shared resources ──────────────────────────────────────────────────────
    auto& sr = UnitreeStateReader::instance();
    if (!sr.start(domain, "")) { std::cerr << "[NavSystem] lowstate reader failed\n"; return false; }
    sr_started_ = true;

    rx_.set_odom_hook([this, &sr](const LioOdometry& od) {
        if (auto st = sr.state_buf.GetData()) prod_.step(od, *st);   // T_odom_pelvis on the Rx thread
    });
    if (!rx_.start(domain)) { std::cerr << "[NavSystem] LIO receiver failed\n"; stop(); return false; }
    rx_started_ = true;

    if (!gr_.start(domain)) { std::cerr << "[NavSystem] goal receiver failed\n"; stop(); return false; }
    gr_started_ = true;

    // Mission goal layer: advertise the named-destination catalog outward, and accept goal
    // commands (by name) inward. Both read the same catalog (config/destinations.yaml).
    const std::string dests_yaml = "config/destinations.yaml";
    if (!destpub_.start(domain, "", dests_yaml)) { std::cerr << "[NavSystem] destination publisher failed\n"; stop(); return false; }
    destpub_started_ = true;
    if (!goalcmd_.start(domain, "", dests_yaml)) { std::cerr << "[NavSystem] goal command receiver failed\n"; stop(); return false; }
    goalcmd_started_ = true;

    if (!pub_.start(domain)) { std::cerr << "[NavSystem] publisher failed\n"; stop(); return false; }
    pub_started_ = true;
    if (!cmd_pub_.start(domain)) { std::cerr << "[NavSystem] cmd publisher failed\n"; stop(); return false; }
    cmd_pub_started_ = true;

    // ── worker config (env) ───────────────────────────────────────────────────
    // Follower cruise speed is env-tunable; terminal dock behavior is per-destination (config).
    FollowConfig fc;
    if (const char* v = std::getenv("NAV_VMAX")) fc.v_max = std::atof(v);
    // Driving is opt-in: only NAV_DRIVE=1 arms the Twist output. Default = preview (no motion).
    const bool drive_enabled = [] { const char* v = std::getenv("NAV_DRIVE"); return v && v[0] == '1'; }();

    // UWB (localization seed only). Non-fatal: no UWB -> the config fixed seed is used.
    if (uwb_.start(domain)) uwb_started_ = true;
    else std::cerr << "[NavSystem] UWB receiver failed — localization seed uses config fallback\n";

    // Localization: relocalize the live odom against the prior PCD -> map->odom. Non-fatal if
    // the prior map is missing — the system still runs (rviz odom goals work; named/map goals
    // stay on hold until a map is present). Seed comes from UWB (recorded map-origin vs live fix)
    // when available, else the config fixed seed.
    {
        const auto  lc = root["localization"];
        const std::string prior_map = lc["prior_map"].as<std::string>("maps/prior_map.pcd");
        const float seed_yaw    = lc["seed"]["yaw_deg"].as<float>(0.f) * float(M_PI) / 180.f;
        const float map_uwb_yaw = lc["map_uwb_yaw_deg"].as<float>(0.f) * float(M_PI) / 180.f;

        // config fixed seed (fallback): translation from seed{x,y}, rotation from seed.yaw_deg
        Eigen::Matrix4f seed = Eigen::Matrix4f::Identity();
        seed.block<3,3>(0,0) = Eigen::AngleAxisf(seed_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        seed(0,3) = lc["seed"]["x"].as<float>(0.f);
        seed(1,3) = lc["seed"]["y"].as<float>(0.f);

        // UWB seed: sidecar (map-origin UWB) sits next to the prior map (.pcd -> .uwb)
        if (uwb_started_) {
            std::string sidecar = prior_map;
            const auto dot = sidecar.rfind(".pcd");
            if (dot != std::string::npos && dot == sidecar.size() - 4) sidecar.replace(dot, 4, ".uwb");
            else sidecar += ".uwb";
            if (uwb_compute_seed(uwb_, sidecar, map_uwb_yaw, seed_yaw, seed))
                std::cout << "[NavSystem] localization seed from UWB\n";
            else
                std::cout << "[NavSystem] localization seed = config fixed (UWB seed unavailable)\n";
        }

        if (!loc_.start(rx_, prior_map, seed, mapodom_buf_))
            std::cerr << "[NavSystem] localization disabled (no prior map at " << prior_map
                      << "); named/map goals need it.\n";
    }

    // ── assemble + launch the workers ─────────────────────────────────────────
    goal_src_.bind(&gr_.goal_buf, &goalcmd_.result(), &mapodom_buf_);
    perc_.start(rx_, prod_, grid_buf_, costmap_buf_);
    plan_.start(costmap_buf_, goal_src_, path_buf_);
    ctrl_.start(path_buf_, prod_, costmap_buf_, goal_src_, cmd_buf_, cmd_pub_, drive_enabled, fc);
    viz_.start(grid_buf_, costmap_buf_, path_buf_, pub_, perc_.gcfg());

    std::cout << "[NavSystem] up: perception + planning + controller + viz (domain " << domain << ").\n";
    if (drive_enabled)
        std::cout << "  *** NAV_DRIVE=1 — Twist IS published: THE ROBOT WILL MOVE. estop ready. ***\n";
    else
        std::cout << "  controller PREVIEW only (no Twist, robot will NOT move). Set NAV_DRIVE=1 to drive.\n";
    std::cout << "  goal: rviz 2D Goal Pose (ad-hoc), or a name on rt/kist/nav/goal "
                 "(catalog: config/destinations.yaml). Ctrl+C to quit.\n";
    return true;
}

void NavSystem::stop() {
    // Workers first — they reference the readers/publishers/buffers below.
    viz_.stop();
    ctrl_.stop();
    plan_.stop();
    perc_.stop();
    loc_.stop();
    if (cmd_pub_started_) { cmd_pub_.stop(); cmd_pub_started_ = false; }   // final zero Twist
    if (uwb_started_) { uwb_.stop(); uwb_started_ = false; }
    if (goalcmd_started_) { goalcmd_.stop(); goalcmd_started_ = false; }
    if (destpub_started_) { destpub_.stop(); destpub_started_ = false; }
    if (gr_started_) { gr_.stop(); gr_started_ = false; }
    if (rx_started_) { rx_.stop(); rx_started_ = false; }
    if (sr_started_) { UnitreeStateReader::instance().stop(); sr_started_ = false; }
    pub_started_ = false;   // publisher stops with its dtor
}

void NavSystem::install_signal_handlers() {
    g_self = this;
    auto h = [](int) { if (g_self) g_self->request_quit(); };
    std::signal(SIGINT,  h);
    std::signal(SIGTERM, h);
}

} // namespace kist
