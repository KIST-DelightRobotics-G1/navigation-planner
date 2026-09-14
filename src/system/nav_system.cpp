#include "system/nav_system.hpp"

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "route_planner/costmap_builder/clearance.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <chrono>
#include <csignal>
#include <iostream>

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

    if (!pub_.start(domain)) { std::cerr << "[NavSystem] publisher failed\n"; stop(); return false; }
    pub_started_ = true;

    running_ = true;
    perc_thread_ = std::thread(&NavSystem::perception_run, this);
    plan_thread_ = std::thread(&NavSystem::planner_run, this);
    viz_thread_  = std::thread(&NavSystem::viz_run, this);

    std::cout << "[NavSystem] up: perception + planning + viz (domain " << domain << ").\n"
              << "  rviz2 -d docs/obstacle_grid.rviz — set a goal with 2D Goal Pose. Ctrl+C to quit.\n";
    return true;
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

void NavSystem::planner_run() {
    while (running_) {
        auto goal = gr_.goal_buf.GetData();
        auto cm   = costmap_buf_.GetData();
        if (goal && cm && !cm->empty())
            path_buf_.SetData(route_.plan(*cm, {cm->robot_x, cm->robot_y}, {goal->x, goal->y}));
        else
            path_buf_.SetData(Path{});
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
    if (viz_thread_.joinable())  viz_thread_.join();
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
