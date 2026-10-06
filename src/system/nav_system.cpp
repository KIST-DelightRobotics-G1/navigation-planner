#include "system/nav_system.hpp"

#include "common/config.hpp"
#include "common/console_tee.hpp"
#include "common/dds_config.hpp"
#include "common/nav_trace.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <string>

namespace kist {

namespace { NavSystem* g_self = nullptr; }

bool NavSystem::start(const std::string& config_path) {
    // Console mirror first, so EVERY line this run prints (including config / DDS / SDK warnings and
    // a crash's last lines) is also on disk at logs/latest.log. Overwritten each run.
    console_started_ = ConsoleTee::instance().start("logs/latest.log");

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
    if (!status_pub_.start(domain)) { std::cerr << "[NavSystem] status publisher failed\n"; stop(); return false; }
    status_pub_started_ = true;

    // ── worker config (env) ───────────────────────────────────────────────────
    // Follower cruise speed is env-tunable; terminal dock behavior is per-destination (config).
    FollowConfig fc;
    if (const char* v = std::getenv("NAV_VMAX")) fc.v_max = std::atof(v);
    // Arrival debounce: ARRIVED is only published after the robot holds at the goal this long
    // (default 1.0 s; env-tunable) so a one-frame flicker never falsely signals arrival.
    double arrival_hold_s = 1.0;
    if (const char* v = std::getenv("NAV_ARRIVAL_HOLD")) arrival_hold_s = std::atof(v);
    // No-path debounce: a transient no-path is reported RUNNING; only a sustained one this long is
    // reported FAILED "no path" (default 4.0 s; env-tunable) so a brief replan gap never aborts a run.
    double nopath_hold_s = 4.0;
    if (const char* v = std::getenv("NAV_NOPATH_HOLD")) nopath_hold_s = std::atof(v);
    // Runtime toggles from config (navigation:). Driving is opt-in: default = preview (no motion).
    bool drive_enabled = false, survey = false;
    if (const auto nav = root["navigation"]) {
        drive_enabled = nav["drive"].as<bool>(false);
        survey        = nav["survey"].as<bool>(false);
    }

    // Episodic-LIO (lio.manage): nav runs the LIO engine only while driving to a goal, so VLA
    // manipulation (which happens while idle) cannot corrupt FAST-LIO. Default off -> LIO is run
    // externally (run_lio_daemon) and runs continuously, exactly as before.
    {
        LioProcessConfig lc;
        if (const auto lio = root["lio"]) {
            lc.manage      = lio["manage"].as<bool>(lc.manage);
            lc.ros_setup   = lio["ros_setup"].as<std::string>(lc.ros_setup);
            lc.ws_setup    = lio["ws_setup"].as<std::string>(lc.ws_setup);
            lc.launch_file = lio["launch_file"].as<std::string>(lc.launch_file);
            lc.log_file    = lio["log_file"].as<std::string>(lc.log_file);
            lc.ready_timeout_s = lio["ready_timeout_s"].as<double>(lc.ready_timeout_s);
        }
        lio_mgr_.configure(lc);
        ready_to_drive_.store(!lc.manage);   // managed: hold until the first goal re-localizes
    }

    // UWB (localization seed only). Non-fatal: no UWB -> the config fixed seed is used.
    if (uwb_.start(domain)) uwb_started_ = true;
    else std::cerr << "[NavSystem] UWB receiver failed — localization seed uses config fallback\n";

    // Localization: yaw-search global_init (UWB xy + robust sweep) seeds a 3-state EKF that then
    // fuses the GICP geometric constraint with the UWB position each cycle -> map->odom. Non-fatal if
    // the prior map is missing (rviz odom goals still work; named/map goals hold until it locks).
    {
        const auto  lc = root["localization"];
        const std::string prior_map = lc["prior_map"].as<std::string>("maps/map.pcd");
        const float seed_yaw    = lc["seed"]["yaw_deg"].as<float>(0.f) * float(M_PI) / 180.f;
        const float map_uwb_yaw = lc["map_uwb_yaw_deg"].as<float>(0.f) * float(M_PI) / 180.f;

        // Fixed fallback seed (used only when no UWB is configured): seed{x,y,yaw_deg}.
        Eigen::Matrix4f fallback = Eigen::Matrix4f::Identity();
        fallback.block<3,3>(0,0) = Eigen::AngleAxisf(seed_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        fallback(0,3) = lc["seed"]["x"].as<float>(0.f);
        fallback(1,3) = lc["seed"]["y"].as<float>(0.f);

        // Sidecar (maps/map.uwb: UWB + P_B) next to the prior map (.pcd -> .uwb).
        std::string sidecar = prior_map;
        const auto dot = sidecar.rfind(".pcd");
        if (dot != std::string::npos && dot == sidecar.size() - 4) sidecar.replace(dot, 4, ".uwb");
        else sidecar += ".uwb";

        // UWB antenna offset in the pelvis/base frame (robot geometry; ~x fwd 3cm, z up 35cm).
        const Eigen::Vector3f tag_in_pelvis(0.03f, 0.f, 0.35f);

        // Relocalizer accept gates (config-tunable without rebuild). Voxel/EKF tuning are header
        // defaults (RelocConfig / LocFilterConfig).
        RelocConfig rcfg;
        rcfg.fitness_max = lc["fitness_max"].as<double>(rcfg.fitness_max);
        rcfg.max_jump_m  = lc["max_jump_m"].as<double>(rcfg.max_jump_m);
        rcfg.max_yaw_deg = lc["max_yaw_deg"].as<double>(rcfg.max_yaw_deg);
        LocFilterConfig fcfg;

        UwbReceiver* uwb_ptr = uwb_started_ ? &uwb_ : nullptr;
        if (!loc_.start(rx_, prod_, uwb_ptr, prior_map, sidecar, map_uwb_yaw, tag_in_pelvis,
                        fallback, mapodom_buf_, rcfg, fcfg, &loc_sample_buf_))
            std::cerr << "[NavSystem] localization disabled (no prior map at " << prior_map
                      << "); named/map goals need it.\n";
    }

    // ── assemble + launch the workers ─────────────────────────────────────────
    goal_src_.bind(&gr_.goal_buf, &goalcmd_.result(), &mapodom_buf_, &leveled_buf_);
    perc_.start(rx_, prod_, grid_buf_, costmap_buf_, leveled_buf_);
    plan_.start(costmap_buf_, goal_src_, path_buf_);
    ctrl_.start(path_buf_, prod_, costmap_buf_, goal_src_, cmd_buf_, cmd_pub_, status_pub_,
                leveled_buf_, drive_enabled, fc, arrival_hold_s, nopath_hold_s,
                &ready_to_drive_, &localize_failed_);
    viz_.start(grid_buf_, costmap_buf_, path_buf_, pub_, perc_.gcfg(), prod_, rx_, mapodom_buf_,
               leveled_buf_, survey, loc_.prior_xyz());   // + T_leveled_odom for rt/leveled_cloud

    // Episodic-LIO coordinator: only when nav manages LIO. Otherwise LIO runs continuously and the
    // gates stay open (ready_to_drive_ = true) — identical to the previous behavior.
    if (lio_mgr_.config().manage) {
        coord_running_ = true;
        coord_thread_  = std::thread(&NavSystem::coordinator_loop, this);
        std::cout << "  *** lio.manage: true — nav starts/stops the LIO engine per goal "
                     "(off while idle). ***\n";
    }

    // Per-tick numeric trace (logs/latest.trace) — a pure observer of the buffers above, filled by
    // the controller thread (zero I/O there; a background thread writes). Default on.
    {
        const auto tr = root["trace"];
        const bool trace_on = !tr || !tr["enabled"] || tr["enabled"].as<bool>(true);
        const std::string trace_path = (tr && tr["path"]) ? tr["path"].as<std::string>()
                                                          : std::string("logs/latest.trace");
        NavTrace::instance().bind(&prod_.out_buf, &leveled_buf_, &path_buf_, &costmap_buf_,
                                  &loc_sample_buf_);
        if (trace_on) trace_started_ = NavTrace::instance().start(trace_path);
    }

    std::cout << "[NavSystem] up: perception + planning + controller + viz (domain " << domain << ").\n";
    if (drive_enabled)
        std::cout << "  *** navigation.drive: true — Twist IS published: THE ROBOT WILL MOVE. estop ready. ***\n";
    else
        std::cout << "  controller PREVIEW only (no Twist, robot will NOT move). Set navigation.drive: true to drive.\n";
    std::cout << "  goal: rviz 2D Goal Pose (ad-hoc), or a SubtaskCmd move_to on rt/cortex/nav/cmd "
                 "(catalog: config/destinations.yaml). state on rt/cortex/nav/state. Ctrl+C to quit.\n";
    return true;
}

// Episodic-LIO mission coordinator (runs only when lio.manage). Detects run start/end from the
// goal (pending_run, independent of localization) and drives the LIO lifecycle + the ready gate:
//   new run  -> start LIO, reinit localization + perception, hold (ready=false) until re-localized
//   localized-> ready=true (controller drives); timeout -> localize_failed (controller reports FAILED)
//   run ends -> stop LIO, close the gate
void NavSystem::coordinator_loop() {
    std::optional<std::pair<std::string, uint16_t>> cur_run;   // (plan_id, index) of the served run
    bool was_pending = false;
    auto run_start = std::chrono::steady_clock::now();
    const double timeout_s = lio_mgr_.config().ready_timeout_s;

    while (coord_running_.load()) {
        const auto pend = goal_src_.pending_run();
        const bool pending = pend.has_value();
        const auto now = std::chrono::steady_clock::now();

        // New run = a goal just became pending, or a DIFFERENT subtask id (not a re-send of the same
        // one — a 5x command burst is one run). request_reinit/reset drop the locks SYNCHRONOUSLY, so
        // the ready check below won't see a stale lock from the previous run.
        if (pending && (!was_pending || *pend != *cur_run)) {        // new run
            cur_run = pend;
            localize_failed_.store(false);
            ready_to_drive_.store(false);
            lio_mgr_.start();                                        // fresh LIO engine
            loc_.request_reinit();                                  // drop lock + submap -> re-localize
            perc_.request_reset();                                  // clear grid + re-estimate tilt
            run_start = now;
            std::cout << "[coordinator] new run -> LIO start + localization/perception reinit\n";
        }

        if (pending && !ready_to_drive_.load() && !localize_failed_.load()) {
            // Drive only once BOTH are freshly re-established: map->odom (localization) AND the
            // gravity-leveled obstacle grid (ground leveler). Driving on a tilted pre-lock grid
            // routes over false obstacles -> no path. (Strict: no identity-grid fallback.)
            if (loc_.locked() && perc_.leveled_locked()) {
                ready_to_drive_.store(true);
                std::cout << "[coordinator] re-localized + leveled -> cleared to drive\n";
            } else if (std::chrono::duration<double>(now - run_start).count() > timeout_s) {
                localize_failed_.store(true);
                std::cerr << "[coordinator] localize/level timed out (" << timeout_s
                          << " s; loc=" << loc_.locked() << " lvl=" << perc_.leveled_locked()
                          << ") -> FAILED\n";
            }
        }

        if (!pending && was_pending) {                              // run ended (arrived / cancelled)
            lio_mgr_.stop();
            ready_to_drive_.store(false);
            localize_failed_.store(false);
            std::cout << "[coordinator] run ended -> LIO stop (idle; safe for manipulation)\n";
        }

        was_pending = pending;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));   // 10 Hz
    }
}

void NavSystem::stop() {
    // Coordinator first (it may start/stop LIO); then workers.
    if (coord_running_.exchange(false) && coord_thread_.joinable()) coord_thread_.join();
    lio_mgr_.stop();                 // ensure the managed LIO engine is down
    // Workers next — they reference the readers/publishers/buffers below.
    viz_.stop();
    ctrl_.stop();
    plan_.stop();
    perc_.stop();
    loc_.stop();
    // Trace after the workers (no more record() calls); it drains the ring and closes the file.
    if (trace_started_) { NavTrace::instance().stop(); trace_started_ = false; }
    if (cmd_pub_started_) { cmd_pub_.stop(); cmd_pub_started_ = false; }   // final zero Twist
    if (status_pub_started_) { status_pub_.stop(); status_pub_started_ = false; }
    if (uwb_started_) { uwb_.stop(); uwb_started_ = false; }
    if (goalcmd_started_) { goalcmd_.stop(); goalcmd_started_ = false; }
    if (destpub_started_) { destpub_.stop(); destpub_started_ = false; }
    if (gr_started_) { gr_.stop(); gr_started_ = false; }
    if (rx_started_) { rx_.stop(); rx_started_ = false; }
    if (sr_started_) { UnitreeStateReader::instance().stop(); sr_started_ = false; }
    pub_started_ = false;   // publisher stops with its dtor
    // Console mirror last, so every shutdown line above is captured before fds 1/2 are restored.
    if (console_started_) { ConsoleTee::instance().stop(); console_started_ = false; }
}

void NavSystem::install_signal_handlers() {
    g_self = this;
    auto h = [](int) { if (g_self) g_self->request_quit(); };
    std::signal(SIGINT,  h);
    std::signal(SIGTERM, h);
}

} // namespace kist
