// Live tuning tool for the RoutePlanner pipeline (grid -> costmap -> A* route -> bubble
// smoother), with env-var overrides so parameters can be swept without rebuilding. The
// deployment binary is ./build/kist-navigation-planner (same pipeline, calibrated defaults,
// no env). Set a goal with rviz's "2D Goal Pose".
//
//   [env...] ./build/test_obstacle_grid [config.yaml]

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "lio/lio_receiver.hpp"
#include "lio/lio_transform_producer.hpp"
#include "mapping/obstacle_grid_publisher.hpp"
#include "planning/goal_receiver.hpp"
#include "route_planner/costmap_builder/clearance.hpp"
#include "route_planner/route_planner.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

int main(int argc, char** argv) {
    const std::string cfg_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(cfg_path);
    const auto& root = Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!apply_dds_config(root)) return 1;
    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    RoutePlanner route;   // calibrated defaults; env overrides below
    // grid / costmap
    if (const char* v = std::getenv("MIN_H"))     route.gcfg.min_height_m           = std::atof(v);
    if (const char* v = std::getenv("UP_MARGIN")) route.gcfg.upper_margin_below_lidar_m = std::atof(v);
    if (const char* v = std::getenv("SLOPE"))     route.gcfg.floor_cut_slope_m_per_m = std::atof(v);
    if (const char* v = std::getenv("DECAY"))     route.gcfg.decay                   = std::atof(v);
    if (const char* v = std::getenv("RES"))       route.gcfg.resolution_m            = std::atof(v);
    if (const char* v = std::getenv("SELF_R"))    route.gcfg.self_radius_m           = std::atof(v);
    if (const char* v = std::getenv("LETHAL_R"))  route.ccfg.lethal_radius_m         = std::atof(v);
    if (const char* v = std::getenv("INFLATE_R")) route.ccfg.influence_radius_m      = std::atof(v);
    // route (A*) + shape (smoother)
    if (const char* v = std::getenv("W_CENTER"))  route.acfg.w_center                = std::atof(v);
    if (const char* v = std::getenv("REF_CLR"))   route.acfg.ref_clr_m               = std::atof(v);
    if (const char* v = std::getenv("D_SAFE"))    route.scfg.d_safe_m                = std::atof(v);
    if (const char* v = std::getenv("R_MIN"))     route.scfg.r_min_m                 = std::atof(v);
    if (const char* v = std::getenv("R_MAX"))     route.scfg.r_max_m                 = std::atof(v);
    if (const char* v = std::getenv("R_STEP"))    route.scfg.r_step_m                = std::atof(v);
    if (const char* v = std::getenv("ANGLE_MIN")) route.scfg.corner_angle_min_deg    = std::atof(v);

    LioTransformProducer prod;
    auto& sr = UnitreeStateReader::instance();
    if (!sr.start(domain, "")) return 1;
    LioReceiver rx;
    rx.set_odom_hook([&](const LioOdometry& od) {
        if (auto st = sr.state_buf.GetData()) prod.step(od, *st);
    });
    if (!rx.start(domain)) return 1;
    GoalReceiver gr;  if (!gr.start(domain)) return 1;
    ObstacleGridPublisher pub;  if (!pub.start(domain)) return 1;

    std::printf("[test_obstacle_grid] RoutePlanner tuning — rviz2 -d docs/obstacle_grid.rviz, 2D Goal Pose. Ctrl+C.\n");

    int64_t last_stamp = 0;
    while (!g_stop) {
        auto scan = rx.cloud_buf.GetData();
        if (!scan || scan->stamp_ns == last_stamp) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
        last_stamp = scan->stamp_ns;
        auto rt = prod.nearest(scan->stamp_ns);
        if (!rt) continue;

        route.update_map(*scan, *rt);
        const ObstacleGrid& grid = route.grid();
        const Costmap&      cm   = route.costmap();
        pub.publish(grid, route.gcfg);
        if (!cm.empty()) {
            pub.publish_costmap(cm);
            const auto clr = clearance_field(cm);
            pub.publish_clearance(cm, clr);
            pub.publish_medial(medial_axis(cm, clr));
        }
        if (auto goal = gr.goal_buf.GetData()) {
            const Path p = route.plan(cm, {grid.robot_x, grid.robot_y}, {goal->x, goal->y});
            pub.publish_path(p.waypoints, route.gcfg.resolution_m);
            pub.publish_path_raw(p.raw_waypoints);
        }
    }

    rx.stop();
    gr.stop();
    sr.stop();
    return 0;
}
