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
#include "labeled_cloud/labeled_cloud_generator.hpp"
#include "labeled_cloud/camera_extrinsics.hpp"
#include "cv/yolo/instance_segmentation/yolo_inst_seg_engine.hpp"
#include "unitree/unitree_pointcloud_reader.hpp"
#include "unitree/unitree_odometry_reader.hpp"
#include "pointcloud/pointcloud_processor.hpp"
#include "kalman_filter/pose_filter.hpp"
#include "system/realsense_receiver.hpp"
#include "system/uwb_receiver.hpp"     // embedded from kist-ext-sensor-io
#include "uwb/uwb_position.hpp"
#include "common/config.hpp"
#include "common/dds_config.hpp"

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
const cv::Scalar kBg(25,25,25), kGrid(55,55,55), kText(225,225,225);

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

GridConfig grid_config_from_yaml(const YAML::Node& root) {
    GridConfig c;
    if (const auto n = root["occupancy_grid"]) {
        c.half_extent_m = n["half_extent_m"].as<float>(c.half_extent_m);
        c.resolution_m  = n["resolution_m"].as<float>(c.resolution_m);
        c.min_z         = n["min_z"].as<float>(c.min_z);
        c.max_z         = n["max_z"].as<float>(c.max_z);
        c.l_hit_lidar   = n["l_hit_lidar"].as<float>(c.l_hit_lidar);
        c.l_hit_cam_near= n["l_hit_cam_near"].as<float>(c.l_hit_cam_near);
        c.l_hit_cam_far = n["l_hit_cam_far"].as<float>(c.l_hit_cam_far);
        c.cam_d_max     = n["cam_d_max"].as<float>(c.cam_d_max);
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

    // ── fusion: clouds + pose -> occupancy grid ──
    const GridConfig gc = grid_config_from_yaml(root);
    OccupancyGridBuilder grid;
    if (!grid.start(lidar.cloud_buf, gen.result(), filter.calibrated_pose_buf, gc)) return 1;

    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    const bool has_disp = [] { const char* e = std::getenv("DISPLAY"); return e && e[0]; }();
    std::printf("[test_occupancy_grid_viewer] domain=%d cam=%s - %s\n"
                "  (move the robot to calibrate the pose; until then the grid is a snapshot)\n",
                domain_id, cam_name.c_str(),
                has_disp ? "window (p = class/probability view, ESC to quit)"
                         : "headless -> /tmp/occupancy_grid.png");

    uint64_t last_processed = 0;
    int      fps = 0;
    bool     show_prob = false;   // toggle with 'p': class view <-> P(occ) heatmap
    auto     window = std::chrono::steady_clock::now();

    while (!g_stop) {
        auto gp = grid.result().GetData();
        auto op = grid.objects().GetData();
        const OccupancyGrid& og = gp ? *gp : OccupancyGrid{};
        const ObjectList&    ol = op ? *op : ObjectList{};
        const bool world = (bool)filter.calibrated_pose_buf.GetData();
        const int  occ = og.empty() ? 0 : count_occupied(og, gc);

        if (has_disp) {
            cv::imshow("occupancy grid (fused, world-anchored)",
                       render(og, gc, ol, world, show_prob, occ, fps));
            const int k = cv::waitKey(30);
            if (k == 27) break;
            if (k == 'p') show_prob = !show_prob;   // class <-> probability view
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

    grid.stop();
    filter.stop();
    uwb.stop();
    odom.stop();
    lidar.stop();
    gen.stop();
    pipe.stop();
    rx.stop();
    return 0;
}
