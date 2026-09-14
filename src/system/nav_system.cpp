#include "system/nav_system.hpp"

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "route_planner/costmap_builder/clearance.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace kist {

namespace { NavSystem* g_self = nullptr; }

bool NavSystem::start(const std::string& config_path) {
    Config::instance().load(config_path);
    const auto& root = Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!apply_dds_config(root)) return false;

    // route_.gcfg/ccfg/acfg/scfg keep their calibrated defaults (real-G1 tuned).

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

    // Env tuning for the follower (sweep without rebuilding). Terminal dock behavior
    // (align/approach/standoff) is per-destination now (config), not env.
    {
        FollowConfig fc = follower_.config();
        if (const char* v = std::getenv("NAV_VMAX")) fc.v_max = std::atof(v);
        follower_.set_config(fc);
    }
    // Driving is opt-in: only NAV_DRIVE=1 arms the Twist output. Default = preview (no motion).
    drive_enabled_ = [] { const char* v = std::getenv("NAV_DRIVE"); return v && v[0] == '1'; }();
    if (!cmd_pub_.start(domain)) { std::cerr << "[NavSystem] cmd publisher failed\n"; stop(); return false; }

    running_ = true;
    perc_thread_ = std::thread(&NavSystem::perception_run, this);
    plan_thread_ = std::thread(&NavSystem::planner_run, this);
    ctrl_thread_ = std::thread(&NavSystem::controller_run, this);
    viz_thread_  = std::thread(&NavSystem::viz_run, this);

    std::cout << "[NavSystem] up: perception + planning + controller + viz (domain " << domain << ").\n";
    if (drive_enabled_)
        std::cout << "  *** NAV_DRIVE=1 — Twist IS published: THE ROBOT WILL MOVE. estop ready. ***\n";
    else
        std::cout << "  controller PREVIEW only (no Twist, robot will NOT move). Set NAV_DRIVE=1 to drive.\n";
    std::cout << "  goal: rviz 2D Goal Pose (ad-hoc), or a name on rt/kist/nav/goal "
                 "(catalog: config/destinations.yaml). Ctrl+C to quit.\n";
    return true;
}

namespace { double quat_yaw2(const Eigen::Quaterniond& q) {
    return std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z())); } }

// Control loop (~20 Hz). Computes the follow command with a fast reactive-stop safety check,
// then EITHER publishes it as a Twist (NAV_DRIVE=1 -> robot moves) OR just previews it. Stops
// (zeros) on stale data, blocked path, no goal, or arrival.
void NavSystem::controller_run() {
    auto last_print = std::chrono::steady_clock::now();
    while (running_) {
        const auto t0 = std::chrono::steady_clock::now();

        NavCommand  cmd;                                    // zeros = stop (default/safe)
        FollowPhase phase = FollowPhase::Arrived;
        auto path = path_buf_.GetDataWithTime();
        auto rtT  = prod_.out_buf.GetDataWithTime();        // freshest robot pose
        auto cm   = costmap_buf_.GetData();
        const bool fresh = path.HasData() && path.GetAgeMs() < 500.0 &&
                           rtT.HasData()  && rtT.GetAgeMs()  < 500.0;
        if (fresh && cm && !cm->empty() && !path.data->empty()) {
            const auto& rt = *rtT.data;
            Pose2D pose{ float(rt.T_odom_pelvis.translation.x()),
                         float(rt.T_odom_pelvis.translation.y()),
                         float(quat_yaw2(rt.T_odom_pelvis.rotation)) };
            float      goal_yaw = std::numeric_limits<float>::quiet_NaN();
            DockConfig dock;                                // default = off (ad-hoc / no goal)
            if (auto g = active_goal(); g && g->valid) {
                dock = g->dock;
                if (g->has_yaw) goal_yaw = g->yaw;          // NaN otherwise -> align skipped
            }
            cmd = follower_.compute(*path.data, pose, dock, cm.get(), goal_yaw, &phase);
            // Reactive sudden-obstacle stop applies ONLY while actively following the path.
            // The terminal approach deliberately creeps toward the goal object (held off by its
            // own front_distance standoff), so path_ahead_blocked — which sees that same object
            // on the path ahead — must not veto it, or the robot stops before reaching standoff.
            if (phase == FollowPhase::Driving &&
                path_ahead_blocked(*cm, *path.data, pose, follower_.config().react_ahead_m,
                                   uint8_t(route_.acfg.obs_cost))) {
                cmd = NavCommand{};                         // sudden obstacle -> stop
                phase = FollowPhase::Blocked;
            }
        }
        cmd_buf_.SetData(cmd);
        if (drive_enabled_) cmd_pub_.publish(cmd);          // Twist -> gearsonic (robot moves)

        if (t0 - last_print >= std::chrono::milliseconds(500)) {
            last_print = t0;
            const char* ps = phase == FollowPhase::Blocked     ? "BLOCKED"
                           : phase == FollowPhase::Aligning    ? "align"
                           : phase == FollowPhase::Approaching ? "approach"
                           : phase == FollowPhase::Arrived     ? "arrived" : "drive";
            std::printf("[controller] vx=% .2f vy=% .2f vyaw=% .2f  %-7s  %s\n",
                        cmd.vx, cmd.vy, cmd.vyaw, ps, drive_enabled_ ? "SENT" : "(preview)");
        }
        std::this_thread::sleep_until(t0 + std::chrono::milliseconds(50));   // ~20 Hz
    }
}

void NavSystem::perception_run() {
    int64_t last_stamp = 0;
    while (running_) {
        auto scan = rx_.cloud_buf.GetData();
        if (!scan || scan->stamp_ns == last_stamp) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        last_stamp = scan->stamp_ns;
        auto rt = prod_.nearest(scan->stamp_ns);
        if (!rt) continue;                            // no pose for this scan
        route_.update_map(*scan, *rt);
        grid_buf_.SetData(route_.grid());
        costmap_buf_.SetData(route_.costmap());
    }
}

// Freshest goal across both channels (whichever DataBuffer was set most recently): rviz
// clicks (gr_, ad-hoc, dock off) and named goal commands (goalcmd_, catalog + dock). Callers
// must check Goal::valid — a cancel command lands as a fresh valid=false goal (= stop).
std::optional<Goal> NavSystem::active_goal() {
    auto a = gr_.goal_buf.GetDataWithTime();
    auto b = goalcmd_.result().GetDataWithTime();
    if (a.HasData() && b.HasData()) return (a.GetAgeMs() <= b.GetAgeMs()) ? *a.data : *b.data;
    if (a.HasData()) return *a.data;
    if (b.HasData()) return *b.data;
    return std::nullopt;
}

void NavSystem::planner_run() {
    while (running_) {
        auto goal = active_goal();
        auto cm   = costmap_buf_.GetData();
        if (goal && goal->valid && cm && !cm->empty())
            path_buf_.SetData(route_.plan(*cm, {cm->robot_x, cm->robot_y}, {goal->x, goal->y}));
        else
            path_buf_.SetData(Path{});   // no / cancelled goal -> empty path -> controller stops
        std::this_thread::sleep_for(std::chrono::milliseconds(200));   // ~5 Hz replan
    }
}

void NavSystem::viz_run() {
    while (running_) {
        if (auto grid = grid_buf_.GetData()) pub_.publish(*grid, route_.gcfg);
        if (auto cm = costmap_buf_.GetData(); cm && !cm->empty()) {
            pub_.publish_costmap(*cm);
            const auto clr = clearance_field(*cm);
            pub_.publish_clearance(*cm, clr);
            pub_.publish_medial(medial_axis(*cm, clr));
        }
        if (auto path = path_buf_.GetData()) {
            pub_.publish_path(path->waypoints, route_.gcfg.resolution_m);
            pub_.publish_path_raw(path->raw_waypoints);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));   // ~10 Hz to rviz
    }
}

void NavSystem::stop() {
    running_ = false;
    if (perc_thread_.joinable()) perc_thread_.join();
    if (plan_thread_.joinable()) plan_thread_.join();
    if (ctrl_thread_.joinable()) ctrl_thread_.join();
    if (viz_thread_.joinable())  viz_thread_.join();
    cmd_pub_.stop();   // final zero Twist (stop) then close — nothing lingers on the wire
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
