// Live validation of the ObstacleGrid core: the registered LIO scan (odom) is
// accumulated into a 3D log-odds voxel grid, 3D-ray-carved from the stamp-matched lidar
// centre, floor removed by a pelvis-referenced height band, then column-projected to 2D.
// A rolling window keeps the grid centred on the robot (voxel_recenter), so the robot can
// walk freely — far cells drop off and new ones come in while accumulation stays valid.
//
//   ./test_obstacle_grid [config.yaml]
//
// View in rviz2 (fixed frame "camera_init"):
//   * Map          topic /obstacle_grid   — the occupancy grid
//   * Pose         topic /robot_pose      — the robot base + HEADING arrow
//   * PointCloud2  topic /cloud_registered_1 — the raw scan, for reference
// A ready config is docs/obstacle_grid.rviz. The floor probe + stats also print to the
// terminal, to calibrate the height band before we lock the floor cut.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "lio/lio_receiver.hpp"
#include "lio/lio_transform_producer.hpp"
#include "mapping/obstacle_grid.hpp"
#include "mapping/obstacle_voxel_grid.hpp"
#include "mapping/costmap.hpp"
#include "mapping/costmap_builder.hpp"
#include "mapping/clearance.hpp"
#include "mapping/obstacle_grid_publisher.hpp"
#include "planning/astar_planner.hpp"
#include "planning/goal_receiver.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

using namespace kist;

static std::atomic<bool> g_stop{false};

namespace {

double quat_yaw(const Eigen::Quaterniond& q) {
    return std::atan2(2*(q.w()*q.z() + q.x()*q.y()),
                      1 - 2*(q.y()*q.y() + q.z()*q.z()));
}

// Floor probe: height distribution of the scan's points near the robot, relative to
// the reference floor (h = p.z - floor_z_odom). If the reference is correct the LOW
// percentiles (the floor returns) sit at ~0. A p05 of -0.15 means the true floor is
// 15 cm below the reference -> raise pelvis_stand_height_m by that much. near_r keeps
// it to the flat area around the robot.
struct FloorProbe { int count = 0; float min_h = 0, p05 = 0, p50 = 0; };
FloorProbe floor_probe(const LioCloud& scan, float ray_ox, float ray_oy,
                       float floor_z, float near_r) {
    std::vector<float> h;
    h.reserve(scan.point_count());
    const float r2 = near_r * near_r;
    const float* p = scan.xyz.data();
    for (std::size_t i = 0; i < scan.point_count(); ++i) {
        const float dx = p[3*i] - ray_ox, dy = p[3*i+1] - ray_oy;
        if (dx*dx + dy*dy > r2) continue;
        h.push_back(p[3*i+2] - floor_z);
    }
    FloorProbe fp;
    if (h.empty()) return fp;
    std::sort(h.begin(), h.end());
    fp.count = int(h.size());
    fp.min_h = h.front();
    fp.p05   = h[std::size_t(0.05f * (h.size() - 1))];
    fp.p50   = h[h.size() / 2];
    return fp;
}

int count_occupied(const ObstacleGrid& g, const ObstacleGridConfig& cfg) {
    int n = 0;
    for (std::size_t i = 0; i < g.log_odds.size(); ++i)
        if (g.prob(int(i)) > cfg.occ_threshold) ++n;
    return n;
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string cfg_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(cfg_path);
    const auto& root = Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!apply_dds_config(root)) return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    ObstacleGridConfig gcfg;   // calibrated defaults live in obstacle_grid.hpp
                               // (pelvis_stand_height 0.89, band [0.12, 1.5])

    // Env overrides — tune without rebuilding. e.g. MIN_H=-1.0 marks the FLOOR as an
    // obstacle (band opened below it); STAND_H shifts the floor plane; MAX_H the ceiling.
    if (const char* v = std::getenv("STAND_H"))  gcfg.pelvis_stand_height_m       = std::atof(v);
    if (const char* v = std::getenv("MIN_H"))    gcfg.min_height_m                = std::atof(v);  // above floor
    if (const char* v = std::getenv("SLOPE"))    gcfg.floor_cut_slope_m_per_m     = std::atof(v);  // floor cut per m range
    if (const char* v = std::getenv("UP_MARGIN"))gcfg.upper_margin_below_lidar_m  = std::atof(v);  // below lidar (dyn top)
    if (const char* v = std::getenv("MAX_H"))    gcfg.max_height_m                = std::atof(v);  // voxel alloc ceiling
    if (const char* v = std::getenv("L_MISS"))   gcfg.l_miss                      = std::atof(v);  // 0 = no free-carving
    if (const char* v = std::getenv("L_HIT"))    gcfg.l_hit                       = std::atof(v);  // hit strength
    if (const char* v = std::getenv("L_CLAMP")){ gcfg.l_max = std::atof(v); gcfg.l_min = -gcfg.l_max; }  // lower = snappier
    if (const char* v = std::getenv("DECAY"))    gcfg.decay                       = std::atof(v);  // ->1.0 = longer memory
    if (const char* v = std::getenv("RES"))      gcfg.resolution_m                = std::atof(v);  // xy cell size
    if (const char* v = std::getenv("RES_Z"))    gcfg.resolution_z_m              = std::atof(v);  // z voxel size
    if (const char* v = std::getenv("SELF_R"))   gcfg.self_radius_m               = std::atof(v);  // self-body radius

    CostmapConfig ccfg;   // lethal/influence radii + cost shape
    if (const char* v = std::getenv("LETHAL_R"))  ccfg.lethal_radius_m    = std::atof(v);  // robot radius
    if (const char* v = std::getenv("INFLATE_R")) ccfg.influence_radius_m = std::atof(v);  // soft zone
    CostmapBuilder cb;
    std::printf("[cfg] stand_h=%.2f  band=[floor+%.2f, lidar-%.2f]  alloc_max=%.2f  l_hit=%.2f l_miss=%.2f decay=%.3f res=%.2f/%.2f\n",
                gcfg.pelvis_stand_height_m, gcfg.min_height_m, gcfg.upper_margin_below_lidar_m,
                gcfg.max_height_m, gcfg.l_hit, gcfg.l_miss, gcfg.decay, gcfg.resolution_m, gcfg.resolution_z_m);

    LioTransformProducer prod;
    auto& sr = UnitreeStateReader::instance();
    if (!sr.start(domain, "")) return 1;

    LioReceiver rx;
    rx.set_odom_hook([&](const LioOdometry& od) {
        if (auto st = sr.state_buf.GetData()) prod.step(od, *st);
    });
    if (!rx.start(domain)) return 1;

    ObstacleGridPublisher pub;
    if (!pub.start(domain)) return 1;

    // Global A* planner + rviz "2D Goal Pose" goal (rt/goal_pose). Click a goal in rviz
    // -> the robot's route is planned on the costmap and published on rt/plan.
    AStarConfig acfg;   // clearance-centred route + LOS straighten + max-radius arc corners
    if (const char* v = std::getenv("W_CENTER"))  acfg.w_center             = std::atof(v);  // route centre preference
    if (const char* v = std::getenv("REF_CLR"))   acfg.ref_clr_m            = std::atof(v);  // "central enough" clearance
    if (const char* v = std::getenv("D_SAFE"))    acfg.d_safe_m             = std::atof(v);  // extra clearance margin
    if (const char* v = std::getenv("R_MIN"))     acfg.r_min_m              = std::atof(v);  // arc radius search
    if (const char* v = std::getenv("R_MAX"))     acfg.r_max_m              = std::atof(v);
    if (const char* v = std::getenv("R_STEP"))    acfg.r_step_m             = std::atof(v);
    if (const char* v = std::getenv("ANGLE_MIN")) acfg.corner_angle_min_deg = std::atof(v);  // straight threshold
    AStarPlanner planner(acfg);
    GoalReceiver gr;
    if (!gr.start(domain)) return 1;

    std::printf("[test_obstacle_grid] publishing /obstacle_grid + /robot_pose (frame camera_init).\n"
                "  rviz2 -d docs/obstacle_grid.rviz — rolling window follows the robot; walk freely. Ctrl+C.\n");

    // floor_z EMA: the floor is static, but the pelvis bobs vertically each step (gait),
    // so pelvis_z - stand_height jitters and the band momentarily dips into the floor,
    // speckling it. Low-passing floor_z holds the band on the true static floor.
    const float floor_ema_a = [&]{ const char* v = std::getenv("FLOOR_EMA"); return v ? float(std::atof(v)) : 0.05f; }();
    float floor_z_ema = 0.f; bool floor_ema_init = false;

    ObstacleVoxelGrid vgrid;   // 3D accumulator
    ObstacleGrid      grid;    // 2D projection (published)
    bool     grid_ready = false;
    int64_t  last_scan_stamp = 0;
    int      no_match = 0;
    FloorProbe fp;
    auto     last_draw = std::chrono::steady_clock::now();
    auto     last_pub  = last_draw;

    while (!g_stop) {
        auto scan = rx.cloud_buf.GetData();
        if (!scan || scan->stamp_ns == last_scan_stamp) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        last_scan_stamp = scan->stamp_ns;

        auto rt = prod.nearest(scan->stamp_ns);
        if (!rt) { ++no_match; continue; }

        const float px = float(rt->T_odom_pelvis.translation.x());
        const float py = float(rt->T_odom_pelvis.translation.y());
        const float lx = float(rt->T_odom_lidar.translation.x());
        const float ly = float(rt->T_odom_lidar.translation.y());
        const float lz = float(rt->T_odom_lidar.translation.z());   // ray origin height (3D carve)
        const float floor_z_raw = float(rt->T_odom_pelvis.translation.z()) - gcfg.pelvis_stand_height_m;
        if (!floor_ema_init) { floor_z_ema = floor_z_raw; floor_ema_init = true; }
        else floor_z_ema += floor_ema_a * (floor_z_raw - floor_z_ema);   // low-pass the gait bob
        const float floor_z = floor_z_ema;

        if (!grid_ready) { voxel_reset(vgrid, gcfg, px, py); grid_ready = true; }

        voxel_recenter(vgrid, px, py);   // rolling window: keep the grid centred on the robot
        voxel_decay(vgrid, gcfg);
        voxel_integrate(vgrid, *scan, lx, ly, lz, floor_z, gcfg);
        voxel_project_to_2d(vgrid, grid);            // 3D column-max -> 2D
        grid.robot_x = px; grid.robot_y = py;
        grid.robot_yaw = float(quat_yaw(rt->T_odom_pelvis.rotation));
        fp = floor_probe(*scan, lx, ly, floor_z, 4.0f);

        const auto now = std::chrono::steady_clock::now();
        if (now - last_pub >= std::chrono::milliseconds(100)) {   // ~10 Hz to rviz
            last_pub = now;
            pub.publish(grid, gcfg);
            const Costmap cm = cb.build(grid, gcfg, ccfg);        // occupancy -> EDT costmap
            pub.publish_costmap(cm);

            // Insight views: clearance heatmap (distance-to-wall) + medial axis (centreline).
            const std::vector<float> clr = clearance_field(cm);
            pub.publish_clearance(cm, clr);
            pub.publish_medial(medial_axis(cm, clr));

            // Plan robot -> clicked goal on the costmap, publish the route (rt/plan).
            if (auto goal = gr.goal_buf.GetData()) {
                const Path route = planner.plan(cm, {px, py}, {goal->x, goal->y});
                pub.publish_path(route.waypoints, gcfg.resolution_m);   // smoothed (bubble)
                pub.publish_path_raw(route.raw_waypoints);              // raw A*
                if (route.empty()) std::printf("\n[plan] no route to (%.2f, %.2f)\n", goal->x, goal->y);
            }
        }
        if (now - last_draw >= std::chrono::seconds(1)) {   // single in-place line, no scroll
            last_draw = now;
            std::printf("\rocc=%-5d  floor p05=%+.2f p50=%+.2f (want ~0 -> stand_h+=%+.2f)  no_match=%d   ",
                        count_occupied(grid, gcfg), fp.p05, fp.p50, -fp.p05, no_match);
            std::fflush(stdout);
        }
    }

    rx.stop();
    sr.stop();
    return 0;
}
