// Live probabilistic semantic occupancy grid — the full perception + localization
// stack, world-anchored.
//   ./test_occupancy_grid_viewer [config_path]      (default config/config.yaml)
// Producers:
//   camera color -> YoloInstPipeline -> mask ┐
//   camera depth ----------------------------> LabeledCloudGenerator ┐
//   LiDAR -> UnitreePointCloudReader(+proc) ---------------------------┤-> OccupancyGridBuilder -> grid
//   odom + UWB -> PoseFilter -> world pose ---------------------------┘
// With a trusted pose the grid is WORLD-anchored and accumulates (P(occ) via
// log-odds, class via camera votes); without pose (calibrating / gated) it falls
// back to a robot-centered snapshot. Cell color = dominant class, brightness =
// P(occ); grey = LiDAR-only (no label). Red dot+arrow = robot pose/heading.
// DISPLAY -> window (ESC); headless -> /tmp/occupancy_grid.png once per second.

#include "occupancy_grid/occupancy_grid_builder.hpp"
#include "planner/planner.hpp"
#include "locomotion/nav_command_publisher.hpp"
#include "labeled_cloud/labeled_cloud_generator.hpp"
#include "labeled_cloud/camera_extrinsics.hpp"
#include "cv/yolo/instance_segmentation/yolo_inst_seg_engine.hpp"
#include "unitree/unitree_pointcloud_reader.hpp"
#include "unitree/unitree_odometry_reader.hpp"
#include "unitree/g1_lowstate_reader.hpp"
#include "pointcloud/pointcloud_processor.hpp"
#include "kalman_filter/pose_filter.hpp"
#include "system/realsense_receiver.hpp"
#include "system/uwb_receiver.hpp"     // embedded from kist-ext-sensor-io
#include "uwb/uwb_position.hpp"
#include "common/config.hpp"
#include "common/dds_config.hpp"

#include <yaml-cpp/yaml.h>
#include <map>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

namespace {
constexpr int kDisp = 700;   // display size, px
const char* kWin = "occupancy grid (fused, world-anchored)";
const cv::Scalar kBg(25,25,25), kGrid(55,55,55), kText(225,225,225);

// A* goal set by clicking the window. The mouse callback runs on the main
// thread (during waitKey), so a plain struct is fine. Geometry is refreshed each
// frame so a click maps display px -> world; the world goal is pushed to the
// planner worker, which plans on its own thread.
struct GoalCtx { bool has = false; float gx = 0, gy = 0;
                 float ox = 0, oy = 0, res = 0.05f; int n = 0; };
GoalCtx   g_ctx;
Planner*  g_planner = nullptr;   // set in main() before the callback is armed

void on_mouse(int event, int x, int y, int, void*) {
    if (event != cv::EVENT_LBUTTONDOWN || g_ctx.n <= 0) return;
    const float pc = float(kDisp) / g_ctx.n;
    const int ix = g_ctx.n - 1 - int(y / pc);   // row = (n-1-ix)*pc
    const int iy = g_ctx.n - 1 - int(x / pc);   // col = (n-1-iy)*pc
    g_ctx.gx = g_ctx.ox + (ix + 0.5f) * g_ctx.res;
    g_ctx.gy = g_ctx.oy + (iy + 0.5f) * g_ctx.res;
    g_ctx.has = true;
    if (g_planner) g_planner->set_goal(g_ctx.gx, g_ctx.gy);
}

// Outline the objects the planner reports as blocking the path (red bbox) and
// banner the count; flag a person (class 0) as the stop-and-ask case.
void draw_blocking(cv::Mat& img, const OccupancyGrid& g,
                   const std::vector<DetectedObject>& blk) {
    if (g.empty() || blk.empty()) return;
    const float pc = float(kDisp) / std::max(1, g.n);
    auto col_px = [&](float wy){ int ix,iy; g.world_to_cell(g.robot_x, wy, ix, iy);
                                 return int((g.n - 1 - iy) * pc); };
    auto row_px = [&](float wx){ int ix,iy; g.world_to_cell(wx, g.robot_y, ix, iy);
                                 return int((g.n - 1 - ix) * pc); };
    bool person = false;
    for (const auto& o : blk) {
        cv::rectangle(img, {col_px(o.max_y), row_px(o.max_x)},
                           {col_px(o.min_y), row_px(o.min_x)}, {0,0,255}, 2);
        if (o.class_id == 0) person = true;
    }
    char t[64];
    std::snprintf(t, sizeof t, "BLOCKED x%zu%s", blk.size(),
                  person ? "  PERSON - ask to pass" : "");
    cv::putText(img, t, {8, kDisp - 12}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,0,255}, 2, cv::LINE_AA);
}

// Draw the A* path (green) + goal marker onto a rendered frame.
void draw_path(cv::Mat& img, const OccupancyGrid& g, const Path& path) {
    if (g.empty()) return;
    const float pc = float(kDisp) / std::max(1, g.n);
    auto to_px = [&](float wx, float wy) {
        int ix, iy; g.world_to_cell(wx, wy, ix, iy);
        return cv::Point(int((g.n - 1 - iy) * pc + pc/2), int((g.n - 1 - ix) * pc + pc/2));
    };
    for (size_t i = 1; i < path.waypoints.size(); ++i)
        cv::line(img, to_px(path.waypoints[i-1].first, path.waypoints[i-1].second),
                      to_px(path.waypoints[i].first,   path.waypoints[i].second),
                 {0, 255, 0}, 2, cv::LINE_AA);
    if (g_ctx.has)
        cv::drawMarker(img, to_px(g_ctx.gx, g_ctx.gy), {0, 255, 0}, cv::MARKER_STAR, 16, 2);
}

// Draw the published base-frame velocity command: a yellow arrow from the robot
// in the commanded direction (base -> world via robot yaw) + a text readout.
void draw_cmd(cv::Mat& img, const OccupancyGrid& g, const NavCommand& c) {
    char t[96];
    std::snprintf(t, sizeof t, "cmd  vx=%.2f  vy=%.2f  vyaw=%.2f", c.vx, c.vy, c.vyaw);
    cv::putText(img, t, {8, kDisp - 34}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 220, 255}, 2, cv::LINE_AA);
    if (g.empty()) return;
    int rix, riy;
    if (!g.world_to_cell(g.robot_x, g.robot_y, rix, riy)) return;
    const float  pc   = float(kDisp) / std::max(1, g.n);
    const int    rcol = int((g.n - 1 - riy + 0.5f) * pc);
    const int    rrow = int((g.n - 1 - rix + 0.5f) * pc);
    const double yaw  = g.robot_yaw;
    const double wdx  = std::cos(yaw) * c.vx - std::sin(yaw) * c.vy;   // world +X
    const double wdy  = std::sin(yaw) * c.vx + std::cos(yaw) * c.vy;   // world +Y
    if (std::hypot(wdx, wdy) < 1e-3) return;
    const double scale = 80.0;   // px per (m/s)
    cv::arrowedLine(img, {rcol, rrow},
                    {rcol - int(wdy * scale), rrow - int(wdx * scale)},   // +Y left, +X up
                    {0, 220, 255}, 2, cv::LINE_AA, 0, 0.3);
}

const cv::Mat& palette() {   // per-id class color (same as the seg viewers)
    static const cv::Mat lut = [] {
        cv::Mat p(256,1,CV_8UC3);
        for (int i=0;i<256;++i){
            cv::Mat hsv(1,1,CV_8UC3,cv::Scalar((i*47)%180,200,255)), bgr;
            cv::cvtColor(hsv,bgr,cv::COLOR_HSV2BGR);
            p.at<cv::Vec3b>(i,0)=bgr.at<cv::Vec3b>(0,0);
        }
        return p;
    }();
    return lut;
}

// World-fixed top-down: world +X up, +Y left. Robot drawn at its world cell.
// show_prob=false: class color, brightness = P(occ). true: pure P(occ) heatmap
// (blue=low .. red=high) over every observed cell, so the belief is visible.
cv::Mat render(const OccupancyGrid& g, const GridConfig& cfg, const ObjectList& objs,
               bool world, bool show_prob, int occ, int fps) {
    cv::Mat img(kDisp, kDisp, CV_8UC3, kBg);
    const float pc = float(kDisp) / std::max(1, g.n);   // px per cell
    if (!g.empty()) {
        cv::Mat cells(g.n, g.n, CV_8UC3, kBg);
        for (int ix = 0; ix < g.n; ++ix)
            for (int iy = 0; iy < g.n; ++iy) {
                const int idx = g.index(ix, iy);
                const float p = g.prob(idx);
                const int row = g.n - 1 - ix, col = g.n - 1 - iy;   // +X up, +Y left
                if (show_prob) {
                    // P(occ) heatmap: only cells that carry occupancy evidence.
                    if (p <= 0.5f) continue;
                    const float t = (p - 0.5f) / 0.48f;             // 0.5->0, ~0.98->1
                    cv::Mat one(1,1,CV_8U,cv::Scalar(uchar(std::clamp(t,0.f,1.f)*255))), jet;
                    cv::applyColorMap(one, jet, cv::COLORMAP_JET);
                    cells.at<cv::Vec3b>(row, col) = jet.at<cv::Vec3b>(0,0);
                } else {
                    if (p <= cfg.occ_threshold) continue;
                    const uint8_t c = g.label[idx];
                    cv::Vec3b base = (c == kNoClass) ? cv::Vec3b(200,200,200)
                                                     : palette().at<cv::Vec3b>(c,0);
                    // brightness spans the visible band (just-occupied dim .. certain bright)
                    const float s = 0.35f + 0.65f *
                        std::clamp((p - cfg.occ_threshold) / (1.f - cfg.occ_threshold), 0.f, 1.f);
                    cells.at<cv::Vec3b>(row, col) =
                        cv::Vec3b(uchar(base[0]*s), uchar(base[1]*s), uchar(base[2]*s));
                }
            }
        cv::resize(cells, img, img.size(), 0, 0, cv::INTER_NEAREST);
    }

    int rix, riy;
    if (!g.empty() && g.world_to_cell(g.robot_x, g.robot_y, rix, riy)) {
        const int rcol = int((g.n - 1 - riy + 0.5f) * pc);
        const int rrow = int((g.n - 1 - rix + 0.5f) * pc);
        const double ppm = kDisp / (2.0 * cfg.half_extent_m);
        for (int r = 1; r <= int(cfg.half_extent_m); ++r)
            cv::circle(img, {rcol,rrow}, int(r*ppm), kGrid, 1, cv::LINE_AA);
        const int tipcol = rcol - int(std::sin(g.robot_yaw) * 0.6 * ppm);   // +Y left
        const int tiprow = rrow - int(std::cos(g.robot_yaw) * 0.6 * ppm);   // +X up
        cv::arrowedLine(img, {rcol,rrow}, {tipcol,tiprow}, cv::Scalar(0,0,255), 2,
                        cv::LINE_AA, 0, 0.3);
        cv::circle(img, {rcol,rrow}, 5, cv::Scalar(0,0,255), -1);
    }

    // Clustered objects: bbox + id/class, colored by class (grey = unlabelled).
    auto world_col = [&](float wy){ int ix,iy; g.world_to_cell(g.robot_x, wy, ix, iy);
                                    return int((g.n - 1 - iy) * pc); };
    auto world_row = [&](float wx){ int ix,iy; g.world_to_cell(wx, g.robot_y, ix, iy);
                                    return int((g.n - 1 - ix) * pc); };
    for (const auto& o : objs.objects) {
        const int c0 = world_col(o.max_y), c1 = world_col(o.min_y);   // +Y left -> min_y right
        const int r0 = world_row(o.max_x), r1 = world_row(o.min_x);   // +X up   -> max_x top
        cv::Scalar col = (o.class_id == kNoClass) ? cv::Scalar(200,200,200)
                         : cv::Scalar(palette().at<cv::Vec3b>(o.class_id,0));
        cv::rectangle(img, {c0,r0}, {c1,r1}, col, 2);
        char t[48]; std::snprintf(t, sizeof t, "#%d c%d", o.id, o.class_id);
        cv::putText(img, t, {c0, r0 - 4}, cv::FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv::LINE_AA);
    }

    char label[160];
    std::snprintf(label, sizeof label, "%s  %d occ  %zu obj  %d fps  (%.0fcm, +-%.0fm, P>%.2f)",
                  world ? "WORLD" : "snapshot (no pose)", occ, objs.size(), fps,
                  cfg.resolution_m*100, cfg.half_extent_m, cfg.occ_threshold);
    cv::putText(img, label, {8,22}, cv::FONT_HERSHEY_SIMPLEX, 0.45, kText, 1, cv::LINE_AA);
    return img;
}

// Costmap view: cost as a faint single-colour tint UNDER the obstacle cells,
// so the inflation gradient reads as a soft background and the actual obstacles
// (class-coloured, on top) stay legible.
cv::Mat render_costmap(const OccupancyGrid& g, const GridConfig& gc,
                       const Costmap& cm, const CostmapConfig& cfg, int fps) {
    cv::Mat img(kDisp, kDisp, CV_8UC3, kBg);
    const cv::Vec3b bg((uchar)kBg[0], (uchar)kBg[1], (uchar)kBg[2]);
    const cv::Vec3b tint(40, 90, 220);   // BGR — one warm hue; opacity ramps with cost
    if (!g.empty()) {
        cv::Mat cells(g.n, g.n, CV_8UC3, kBg);
        // 1) cost tint underneath (fades from bg at low cost to the hue at lethal)
        if (!cm.empty())
            for (int ix = 0; ix < g.n; ++ix)
                for (int iy = 0; iy < g.n; ++iy) {
                    const uint8_t c = cm.at(ix, iy);
                    if (c == 0) continue;
                    const float a = std::min(1.f, float(c) / cfg.lethal_cost) * 0.7f;  // faint
                    const int row = g.n - 1 - ix, col = g.n - 1 - iy;
                    cells.at<cv::Vec3b>(row, col) = cv::Vec3b(
                        uchar(bg[0]*(1-a) + tint[0]*a),
                        uchar(bg[1]*(1-a) + tint[1]*a),
                        uchar(bg[2]*(1-a) + tint[2]*a));
                }
        // 2) obstacle cells ON TOP (class colour), so points stay visible
        for (int ix = 0; ix < g.n; ++ix)
            for (int iy = 0; iy < g.n; ++iy) {
                const int idx = g.index(ix, iy);
                if (g.prob(idx) <= gc.occ_threshold) continue;
                const uint8_t c = g.label[idx];
                cells.at<cv::Vec3b>(g.n-1-ix, g.n-1-iy) =
                    (c == kNoClass) ? cv::Vec3b(200,200,200) : palette().at<cv::Vec3b>(c,0);
            }
        cv::resize(cells, img, img.size(), 0, 0, cv::INTER_NEAREST);
    }
    int rix, riy;
    if (!g.empty() && g.world_to_cell(g.robot_x, g.robot_y, rix, riy)) {
        const float pc = float(kDisp) / std::max(1, g.n);
        cv::circle(img, {int((g.n-1-riy+0.5f)*pc), int((g.n-1-rix+0.5f)*pc)}, 5, {0,0,255}, -1);
    }
    char label[120];
    std::snprintf(label, sizeof label, "COSTMAP  lethal=%.2fm influence=%.2fm  %d fps",
                  cfg.lethal_radius_m, cfg.influence_radius_m, fps);
    cv::putText(img, label, {8,22}, cv::FONT_HERSHEY_SIMPLEX, 0.45, kText, 1, cv::LINE_AA);
    return img;
}

CostmapConfig costmap_config_from_yaml(const YAML::Node& root) {
    CostmapConfig c;
    if (const auto n = root["costmap"]) {
        c.lethal_radius_m     = n["lethal_radius_m"].as<float>(c.lethal_radius_m);
        c.influence_radius_m  = n["influence_radius_m"].as<float>(c.influence_radius_m);
        c.lethal_cost         = uint8_t(n["lethal_cost"].as<int>(c.lethal_cost));
        c.max_soft_cost       = uint8_t(n["max_soft_cost"].as<int>(c.max_soft_cost));
        c.decay               = n["decay"].as<float>(c.decay);
        c.ignore_dynamic      = n["ignore_dynamic"].as<bool>(c.ignore_dynamic);
        c.dynamic_inflation_m = n["dynamic_inflation_m"].as<float>(c.dynamic_inflation_m);
        c.obstacle_min_neighbors = n["obstacle_min_neighbors"].as<int>(c.obstacle_min_neighbors);
    }
    return c;
}

AStarConfig astar_config_from_yaml(const YAML::Node& root) {
    AStarConfig c;
    if (const auto n = root["astar"]) {
        c.base_cost        = n["base_cost"].as<float>(c.base_cost);
        c.obs_cost         = n["obs_cost"].as<int>(c.obs_cost);
        c.heuristic_weight = n["heuristic_weight"].as<float>(c.heuristic_weight);
        c.max_iterations   = n["max_iterations"].as<int>(c.max_iterations);
        c.trust_falloff_m  = n["trust_falloff_m"].as<float>(c.trust_falloff_m);
        c.trust_min        = n["trust_min"].as<float>(c.trust_min);
        c.max_path_len_m   = n["max_path_len_m"].as<float>(c.max_path_len_m);
        c.goal_ignore_radius_m = n["goal_ignore_radius_m"].as<float>(c.goal_ignore_radius_m);
    }
    return c;
}

FollowConfig follow_config_from_yaml(const YAML::Node& root) {
    FollowConfig c;
    if (const auto nav = root["navigation"])
        if (const auto n = nav["follow"]) {
            c.v_max         = n["v_max"].as<double>(c.v_max);
            c.vyaw_max      = n["vyaw_max"].as<double>(c.vyaw_max);
            c.lookahead_m   = n["lookahead_m"].as<double>(c.lookahead_m);
            c.arrival_tol_m = n["arrival_tol_m"].as<double>(c.arrival_tol_m);
            c.yaw_tol_rad   = n["yaw_tol_deg"].as<double>(c.yaw_tol_rad * 180.0 / M_PI) * M_PI / 180.0;
            c.k_yaw         = n["k_yaw"].as<double>(c.k_yaw);
            c.slow_radius_m = n["slow_radius_m"].as<double>(c.slow_radius_m);
            c.rate_hz       = n["rate_hz"].as<double>(c.rate_hz);
            c.allow_strafe  = n["allow_strafe"].as<bool>(c.allow_strafe);
        }
    return c;
}

// Named destinations from destinations.yaml (world frame; yaw = finish heading).
struct Dest { float x = 0, y = 0, yaw_deg = 0; };
std::map<std::string, Dest> load_destinations(const std::string& path) {
    std::map<std::string, Dest> out;
    try {
        const YAML::Node root = YAML::LoadFile(path);
        if (const auto ds = root["destinations"])
            for (const auto& d : ds)
                out[d["name"].as<std::string>("")] =
                    Dest{d["x"].as<float>(0), d["y"].as<float>(0), d["yaw_deg"].as<float>(0)};
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[dest] load failed (%s): %s\n", path.c_str(), e.what());
    }
    return out;
}

GridConfig grid_config_from_yaml(const YAML::Node& root) {
    GridConfig c;
    if (const auto n = root["occupancy_grid"]) {
        c.half_extent_m = n["half_extent_m"].as<float>(c.half_extent_m);
        c.resolution_m  = n["resolution_m"].as<float>(c.resolution_m);
        c.min_z         = n["min_z"].as<float>(c.min_z);
        c.max_z         = n["max_z"].as<float>(c.max_z);
        c.level_by_odom = n["level_by_odom"].as<bool>(c.level_by_odom);
        c.use_waist_fk  = n["use_waist_fk"].as<bool>(c.use_waist_fk);
        c.l_hit_lidar   = n["l_hit_lidar"].as<float>(c.l_hit_lidar);
        c.l_hit_cam_near= n["l_hit_cam_near"].as<float>(c.l_hit_cam_near);
        c.l_hit_cam_far = n["l_hit_cam_far"].as<float>(c.l_hit_cam_far);
        c.cam_d_max     = n["cam_d_max"].as<float>(c.cam_d_max);
        c.l_miss_lidar  = n["l_miss_lidar"].as<float>(c.l_miss_lidar);
        c.camera_occupancy = n["camera_occupancy"].as<bool>(c.camera_occupancy);
        c.decay_static  = n["decay_static"].as<float>(c.decay_static);
        c.decay_dynamic = n["decay_dynamic"].as<float>(c.decay_dynamic);
        c.occ_threshold = n["occ_threshold"].as<float>(c.occ_threshold);
        c.cluster_min_area_m2 = n["cluster_min_area_m2"].as<float>(c.cluster_min_area_m2);
        c.cluster_morph_cells = n["cluster_morph_cells"].as<int>(c.cluster_morph_cells);
        c.cluster_min_labeled_frac = n["cluster_min_labeled_frac"].as<float>(c.cluster_min_labeled_frac);
        c.cluster_label_min_frac   = n["cluster_label_min_frac"].as<float>(c.cluster_label_min_frac);
        c.cluster_drop_unknown_frac = n["cluster_drop_unknown_frac"].as<float>(c.cluster_drop_unknown_frac);
        c.cluster_footprint_pad_m  = n["cluster_footprint_pad_m"].as<float>(c.cluster_footprint_pad_m);
        if (const auto dc = n["dynamic_classes"])
            for (const auto& id : dc) {
                const int v = id.as<int>(-1);
                if (v >= 0 && v < 256) c.is_dynamic[v] = true;
            }
    }
    return c;
}

int count_occupied(const OccupancyGrid& g, const GridConfig& cfg) {
    int n = 0;
    for (size_t i = 0; i < g.log_odds.size(); ++i)
        if (g.prob(int(i)) > cfg.occ_threshold) ++n;
    return n;
}
}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    Config::instance().load(config_path);
    const auto& root = Config::instance().root();

    const int domain_id = root["unitree"]["domain_id"].as<int>(0);
    if (!apply_dds_config(root)) return 1;

    // Optional named destination (argv[2]): looked up in destinations.yaml and
    // set as the goal once the pose is WORLD — drives there without a click.
    const std::string dest_name = (argc >= 3) ? argv[2] : "";
    Dest goal_dest;
    bool have_dest = false, go_requested = false;
    if (!dest_name.empty()) {
        std::string dfile = "config/destinations.yaml";
        if (const auto nav = root["navigation"])
            dfile = nav["destinations_file"].as<std::string>(dfile);
        const auto dests = load_destinations(dfile);
        if (const auto it = dests.find(dest_name); it != dests.end()) {
            goal_dest = it->second; have_dest = true;
            std::printf("[dest] target '%s' at (%.2f, %.2f) yaw %.0f deg — calibrate, then press 'g' to GO\n",
                        dest_name.c_str(), goal_dest.x, goal_dest.y, goal_dest.yaw_deg);
        } else {
            std::fprintf(stderr, "[dest] '%s' not found in %s\n", dest_name.c_str(), dfile.c_str());
        }
    }

    // ── camera: color -> YOLO, depth -> labeled cloud ──
    YoloInstSegConfig ycfg;
    double      target_fps = 30.0;
    std::string cam_name   = "head";
    if (const auto cv = root["cv_inference"]) {
        ycfg.onnx_path = cv["instance_onnx"].as<std::string>(ycfg.onnx_path);
        ycfg.score_threshold = cv["score_threshold"].as<float>(ycfg.score_threshold);
        ycfg.mask_erode_px = cv["mask_erode_px"].as<int>(ycfg.mask_erode_px);
        target_fps     = cv["target_fps"].as<double>(target_fps);
        cam_name       = cv["camera"].as<std::string>(cam_name);
    }
    LabeledCloudConfig lcfg;
    if (const auto f = root["fusion"]) {
        if (const auto e = f["camera_extrinsics"])
            lcfg.extrinsics = make_camera_extrinsics(
                e["x"].as<float>(0), e["y"].as<float>(0), e["height"].as<float>(0.8f),
                e["pitch_deg"].as<float>(45.f), e["yaw_deg"].as<float>(0), e["roll_deg"].as<float>(0));
        if (const auto g = f["ground_plane"]) {
            lcfg.ground.enabled         = g["enabled"].as<bool>(lcfg.ground.enabled);
            lcfg.ground.inlier_eps_m    = g["inlier_eps_m"].as<float>(lcfg.ground.inlier_eps_m);
            lcfg.ground.max_tilt_deg    = g["max_tilt_deg"].as<float>(lcfg.ground.max_tilt_deg);
            lcfg.ground.candidate_z_max = g["candidate_z_max"].as<float>(lcfg.ground.candidate_z_max);
            lcfg.ground.iterations      = g["iterations"].as<int>(lcfg.ground.iterations);
            lcfg.ground.min_inliers     = g["min_inliers"].as<int>(lcfg.ground.min_inliers);
            lcfg.ground.smooth          = g["smooth"].as<float>(lcfg.ground.smooth);
        }
    }

    RealsenseReceiver rx;
    if (!rx.start(domain_id, "", cam_name)) return 1;

    YoloInstPipeline pipe;
    if (!pipe.start(ycfg, target_fps, [&](cv::Mat& bgr, int64_t& stamp) -> bool {
        auto cf = rx.color().GetData();
        if (!cf || cf->empty()) return false;
        stamp = cf->stamp_ns;
        cv::Mat(cf->height, cf->width, CV_8UC3,
                const_cast<uint8_t*>(cf->data.data()), cf->stride_bytes).copyTo(bgr);
        return true;
    })) return 1;

    LabeledCloudGenerator gen;
    if (!gen.start(rx.depth(), pipe.result(), lcfg)) return 1;

    // ── LiDAR: reader + processor (into the robot/planning frame) ──
    auto& lidar = UnitreePointCloudReader::instance();
    lidar.set_processor(
        [proc = PointCloudProcessor{
             pointcloud_processor_options_from_yaml(root["pointcloud_processor"])}](
            UnitreePointCloud& cloud) mutable { proc.process(cloud); });
    if (!lidar.start(domain_id, "")) return 1;

    // ── localization: odom + UWB -> PoseFilter -> world pose ──
    auto& odom = UnitreeOdometryReader::instance();
    UwbReceiver uwb;
    PoseFilter filter(pose_filter_options_from_yaml(root["pose_filter"]));
    uwb.set_on_position([&](const UwbPosition& p) {
        filter.uwb_buf.SetData(UwbFix{p.stamp_ns, p.x, p.y});
    });
    if (!odom.start(domain_id, "")) return 1;
    if (!uwb.start(domain_id, "")) return 1;
    filter.start(odom.odom_buf);

    // G1 waist joints -> FK torso-bob leveling in the grid builder.
    auto& lowstate = G1LowStateReader::instance();
    if (!lowstate.start(domain_id, "")) return 1;

    // ── fusion: clouds + pose -> occupancy grid ──
    const GridConfig gc = grid_config_from_yaml(root);
    ScanMatchConfig smc;
    if (const auto og2 = root["occupancy_grid"])
        if (const auto sm = og2["scan_match"]) {
            smc.enabled         = sm["enabled"].as<bool>(smc.enabled);
            smc.search_xy_m     = sm["search_xy_m"].as<float>(smc.search_xy_m);
            smc.search_yaw_deg  = sm["search_yaw_deg"].as<float>(smc.search_yaw_deg);
            smc.step_xy_m       = sm["step_xy_m"].as<float>(smc.step_xy_m);
            smc.yaw_steps       = sm["yaw_steps"].as<int>(smc.yaw_steps);
            smc.sigma_m         = sm["sigma_m"].as<float>(smc.sigma_m);
            smc.min_scan_points = sm["min_scan_points"].as<int>(smc.min_scan_points);
            smc.min_map_cells   = sm["min_map_cells"].as<int>(smc.min_map_cells);
        }
    OccupancyGridBuilder grid;
    if (!grid.start(lidar.cloud_buf, gen.result(), filter.calibrated_pose_buf, odom.odom_buf,
                    lowstate.waist_buf, gc, smc)) return 1;

    // ── planning: grid -> costmap -> A* path, on its own worker thread ──
    const CostmapConfig cc = costmap_config_from_yaml(root);   // kept for the cost-view label
    PlannerConfig pcfg{gc, cc, astar_config_from_yaml(root)};
    if (const auto pn = root["planner"]) {
        pcfg.replan_deviation_m    = pn["replan_deviation_m"].as<float>(pcfg.replan_deviation_m);
        pcfg.replan_cost_threshold = pn["replan_cost_threshold"].as<float>(pcfg.replan_cost_threshold);
    }
    Planner planner;
    if (!planner.start(grid.result(), grid.objects(), pcfg)) return 1;
    g_planner = &planner;   // arm the mouse callback's goal push

    // ── locomotion: path + pose -> base-frame (vx,vy,vyaw) Twist over DDS ──
    const FollowConfig fcfg = follow_config_from_yaml(root);
    std::string cmd_topic = kNavCmdTopic;
    if (const auto nav = root["navigation"])
        cmd_topic = nav["cmd_vel_topic"].as<std::string>(cmd_topic);
    NavCommandPublisher follower;
    if (!follower.start(domain_id, "", planner.result(), filter.calibrated_pose_buf, fcfg, cmd_topic))
        return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    const bool has_disp = [] { const char* e = std::getenv("DISPLAY"); return e && e[0]; }();
    if (has_disp) {
        cv::namedWindow(kWin);
        cv::setMouseCallback(kWin, on_mouse);   // left-click sets the A* goal
    }
    std::printf("[test_occupancy_grid_viewer] domain=%d cam=%s - %s\n"
                "  (move the robot to calibrate the pose; until then the grid is a snapshot)\n",
                domain_id, cam_name.c_str(),
                has_disp ? "window (p = prob, c = costmap, click = A* goal, x = clear, ESC to quit)"
                         : "headless -> /tmp/occupancy_grid.png");

    uint64_t last_processed = 0;
    int      fps = 0;
    bool     show_prob = false;   // toggle with 'p': class view <-> P(occ) heatmap
    bool     show_cost = false;   // toggle with 'c': costmap view
    auto     window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto gp = grid.result().GetData();
        auto op = grid.objects().GetData();
        const OccupancyGrid& og = gp ? *gp : OccupancyGrid{};
        const ObjectList&    ol = op ? *op : ObjectList{};
        const bool world = (bool)filter.calibrated_pose_buf.GetData();
        const int  occ = og.empty() ? 0 : count_occupied(og, gc);

        // Arrived (reached goal + aligned) -> clear the destination: the robot
        // stops and the goal marker disappears, so arrival is obvious.
        if (g_ctx.has && follower.phase() == NavCommandPublisher::Phase::Arrived) {
            planner.clear_goal();
            follower.clear_goal_yaw();
            g_ctx.has = false;
            std::printf("[dest] arrived — destination cleared\n");
        }

        // Named destination: issued ONLY on an explicit 'g' press (and once the
        // pose is world-anchored — world coords only line up then). No auto-start,
        // so the robot won't lurch the moment the stick goes neutral after calib.
        if (have_dest && go_requested && world) {
            planner.set_goal(goal_dest.x, goal_dest.y);
            follower.set_goal_yaw(goal_dest.yaw_deg * float(M_PI) / 180.0f);
            g_ctx.gx = goal_dest.x; g_ctx.gy = goal_dest.y; g_ctx.has = true;
            go_requested = false;
            std::printf("[dest] GO -> '%s' (%.2f, %.2f)\n",
                        dest_name.c_str(), goal_dest.x, goal_dest.y);
        }

        if (has_disp) {
            // keep the click->world mapping current for this frame's geometry
            g_ctx.ox = og.origin_x; g_ctx.oy = og.origin_y;
            g_ctx.res = og.resolution; g_ctx.n = og.n;

            // The planner worker owns costmap + A*; just read its latest output.
            auto pp = planner.result().GetData();
            cv::Mat frame = show_cost
                ? render_costmap(og, gc, pp ? pp->costmap : Costmap{}, cc, fps)
                : render(og, gc, ol, world, show_prob, occ, fps);
            draw_path(frame, og, pp ? pp->path : Path{});
            if (pp) draw_blocking(frame, og, pp->blocking);
            if (auto np = follower.result().GetData()) draw_cmd(frame, og, *np);
            cv::imshow(kWin, frame);
            const int k = cv::waitKey(30);
            if (k == 27) break;
            if (k == 'p') show_prob = !show_prob;   // class <-> probability view
            if (k == 'c') show_cost = !show_cost;   // grid <-> costmap view
            if (k == 'm') {                                             // toggle scan matching (A/B)
                grid.set_scan_match(!grid.scan_match_on());
                std::printf("[scan_match] %s\n", grid.scan_match_on() ? "ON" : "OFF");
            }
            if (k == 'x') { g_ctx.has = false; planner.clear_goal(); }  // clear the A* goal
            if (k == 'g' && have_dest) {                                 // request GO to the named destination
                go_requested = true;
                if (!world) std::printf("[dest] GO queued — waiting for WORLD (calibrate first)\n");
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - window >= std::chrono::seconds(1)) {
            window = now;
            const uint64_t p = grid.frames_processed();
            fps = int(p - last_processed); last_processed = p;
            std::printf("  %s  %d occupied cells  %zu objects  %d fps\n",
                        world ? "WORLD" : "snapshot", occ, ol.size(), fps);
            if (!has_disp) cv::imwrite("/tmp/occupancy_grid.png",
                                       render(og, gc, ol, world, show_prob, occ, fps));
        }
    }

    follower.stop();
    planner.stop();
    grid.stop();
    lowstate.stop();
    filter.stop();
    uwb.stop();
    odom.stop();
    lidar.stop();
    gen.stop();
    pipe.stop();
    rx.stop();
    return 0;
}
