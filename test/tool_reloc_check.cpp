// reloc_check — v0 validation for the Relocalizer (map->odom via GICP against a prior map).
//
// Loads the prior PCD, accumulates live registered scans into a rolling submap, and repeatedly
// GICP-aligns the submap to the prior from a manual seed. Prints the fitness + resulting
// T_map_odom (x, y, yaw) and dumps reloc_overlay.pcd (prior + submap-transformed-by-T_map_odom)
// so you can eyeball the overlap in pcl_viewer. This isolates FRAME/registration correctness
// before wiring map->odom into the planner.
//
//   SEED_X=.. SEED_Y=.. SEED_YAW_DEG=.. ./build/reloc_check [prior.pcd=maps/prior_map.pcd] [config]
//   -> stand/drive near the seed pose; watch fitness drop + overlap tighten. Ctrl+C to stop.
//   pcl_viewer reloc_overlay.pcd     # prior (dark) vs relocalized submap (bright) should coincide
//
// GLOBAL_INIT mode (Phase 1 yaw-search init — no trusted seed yaw, arbitrary boot heading):
//   GLOBAL_INIT=1 ./build/reloc_check     # UWB fix + T_odom_pelvis are received LIVE
//   -> accumulates a submap, then runs Relocalizer::global_init: sweeps yaw, prints the coarse
//      inlier-ratio curve + best/2nd/confidence, and (on accept) seeds tracking. Tune with
//      INLIER_THRESH / YAW_STEP / MIN_INLIER / MIN_CONF / PEAK_SEP.
//      The UWB tag xy in the map is computed from the live fix vs the recorded map-origin sidecar
//      (maps/prior_map.uwb) + localization.map_uwb_yaw_deg. Offline replay override: set UWB_X/UWB_Y
//      (tag xy in map) and/or ODOM_X/Y/Z/YAW_DEG (odom<-base) to bypass the live sources.

#include "common/config.hpp"
#include "common/dds_config.hpp"
#include "lio/lio_receiver.hpp"
#include "lio/lio_transform_producer.hpp"
#include "localization/relocalizer.hpp"
#include "localization/uwb_receiver.hpp"
#include "mapping/obstacle_grid_publisher.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace kist;

namespace {
std::atomic<bool> g_stop{false};

float envf(const char* k, float dflt) { const char* v = std::getenv(k); return v ? float(std::atof(v)) : dflt; }

Eigen::Matrix4f seed_from_xyyaw(float x, float y, float yaw) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    T(0,3) = x; T(1,3) = y;
    return T;
}
float yaw_of(const Eigen::Matrix4f& T) { return std::atan2(T(1,0), T(0,0)); }

// Non-blocking raw-terminal keyboard for the MANUAL yaw slider (restores termios on destruction).
struct RawKb {
    termios old{}; bool on = false;
    RawKb() {
        if (tcgetattr(STDIN_FILENO, &old) != 0) return;
        termios raw = old;
        raw.c_lflag &= ~(ICANON | ECHO);
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return;
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
        on = true;
    }
    ~RawKb() { if (on) tcsetattr(STDIN_FILENO, TCSANOW, &old); }
    int get() { char c; return (read(STDIN_FILENO, &c, 1) == 1) ? (unsigned char)c : -1; }  // -1 = none
};
}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string prior_path = (argc >= 2) ? argv[1] : "maps/map.pcd";
    const std::string cfg_path   = (argc >= 3) ? argv[2] : "config/config.yaml";

    Config::instance().load(cfg_path);
    const auto& root = Config::instance().root();
    const int domain = root["unitree"]["domain_id"].as<int>(0);
    if (!apply_dds_config(root)) return 1;
    std::signal(SIGINT,  [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    Relocalizer reloc;
    {   // env-tunable knobs (sweep without rebuilding)
        RelocConfig rc = reloc.config();
        rc.map_voxel_m    = envf("MAP_VOXEL",   rc.map_voxel_m);
        rc.submap_voxel_m = envf("SUB_VOXEL",   rc.submap_voxel_m);
        rc.submap_scans   = int(envf("SUB_SCANS", float(rc.submap_scans)));
        rc.gicp_max_corr_m= envf("MAX_CORR",    float(rc.gicp_max_corr_m));
        rc.fitness_max    = envf("FITNESS_MAX", float(rc.fitness_max));
        rc.max_jump_m     = envf("MAX_JUMP",    float(rc.max_jump_m));
        // global yaw-search init knobs
        rc.gi_inlier_thresh_m  = envf("INLIER_THRESH", rc.gi_inlier_thresh_m);
        rc.gi_yaw_step_deg     = envf("YAW_STEP",      rc.gi_yaw_step_deg);
        rc.gi_min_inlier_ratio = envf("MIN_INLIER",    rc.gi_min_inlier_ratio);
        rc.gi_min_confidence   = envf("MIN_CONF",      rc.gi_min_confidence);
        rc.gi_peak_sep_deg     = envf("PEAK_SEP",      rc.gi_peak_sep_deg);
        rc.gi_floor_filter     = envf("FLOOR_FILTER",  rc.gi_floor_filter ? 1.f : 0.f) != 0.f;
        rc.gi_floor_band_m     = envf("FLOOR_BAND",    rc.gi_floor_band_m);
        rc.gi_wall_top_m       = envf("WALL_TOP",      rc.gi_wall_top_m);
        reloc.set_config(rc);
    }
    if (!reloc.load_prior(prior_path)) return 1;

    const bool global_mode = envf("GLOBAL_INIT", 0.f) != 0.f;
    const bool no_lock     = envf("NO_LOCK", 0.f) != 0.f;      // keep scoring; never switch to tracking
    const bool print_curve = envf("PRINT_CURVE", 0.f) != 0.f;  // dump the full coarse yaw curve
    const bool manual_mode = envf("MANUAL", 0.f) != 0.f;       // keyboard yaw slider instead of auto sweep
    const Eigen::Vector3f tag_in_pelvis(envf("OFF_X", 0.03f), envf("OFF_Y", 0.f), envf("OFF_Z", 0.35f));
    // Offline replay overrides (bypass the live sources): UWB_X/Y = tag xy in map; ODOM_* = odom<-base.
    const bool env_uwb  = std::getenv("UWB_X") != nullptr;
    const bool env_odom = std::getenv("ODOM_YAW_DEG") || std::getenv("ODOM_X") || std::getenv("ODOM_Y");
    const float map_uwb_yaw = root["localization"]["map_uwb_yaw_deg"].as<float>(0.f) * float(M_PI) / 180.f;
    std::string sidecar = prior_path;                              // maps/prior_map.pcd -> .uwb
    { const auto d = sidecar.rfind(".pcd");
      if (d != std::string::npos && d == sidecar.size()-4) sidecar.replace(d,4,".uwb"); else sidecar += ".uwb"; }

    if (!global_mode) {
        const float sx = envf("SEED_X", 0.f), sy = envf("SEED_Y", 0.f);
        const float syaw = envf("SEED_YAW_DEG", 0.f) * float(M_PI) / 180.f;
        reloc.seed(seed_from_xyyaw(sx, sy, syaw));
        std::printf("[reloc_check] seed T_map_odom: x=%.2f y=%.2f yaw=%.1f deg\n", sx, sy, syaw*57.2958f);
    } else {
        std::printf("[reloc_check] GLOBAL_INIT mode: %s UWB, %s odom<-base, offset=(%.2f,%.2f,%.2f); "
                    "accumulating submap, will yaw-search when ready...\n",
                    env_uwb ? "env" : "live", env_odom ? "env" : "live",
                    tag_in_pelvis.x(), tag_in_pelvis.y(), tag_in_pelvis.z());
    }

    // Live sources for global init: the LIO transform producer needs lowstate (waist FK) to make
    // T_odom_pelvis, and the UWB receiver gives the tag fix. Only needed in live global mode.
    auto& sr = UnitreeStateReader::instance();
    LioTransformProducer prod;
    UwbReceiver uwb;
    const bool live = global_mode && !(env_uwb && env_odom);
    if (live) {
        if (!sr.start(domain, "")) { std::fprintf(stderr, "[reloc_check] lowstate reader failed\n"); return 1; }
        if (!uwb.start(domain))    std::fprintf(stderr, "[reloc_check] UWB receiver failed (use UWB_X/Y)\n");
    }

    LioReceiver rx;
    if (live)
        rx.set_odom_hook([&](const LioOdometry& od) {             // T_odom_pelvis on the Rx thread
            if (auto st = sr.state_buf.GetData()) prod.step(od, *st);
        });
    if (!rx.start(domain)) { std::fprintf(stderr, "[reloc_check] LIO receiver failed\n"); return 1; }
    std::printf("[reloc_check] receiving rt/cloud_registered_1; building submap...\n");

    // rviz2 overlay of the best hypothesis (map frame): prior on rt/prior_map, submap@best on
    // rt/cloud_sway. Fixed frame camera_init; if the yaw is right the submap walls sit on the prior.
    ObstacleGridPublisher viz_pub;
    const bool viz = global_mode && viz_pub.start(domain);
    if (viz) std::printf("[reloc_check] rviz2: add PointCloud2 rt/prior_map (prior) + rt/cloud_sway "
                         "(submap@best), fixed frame camera_init.\n");

    // Resolve the global-init inputs (live UWB tag xy + odom<-base, or env overrides). Shared by
    // the auto-sweep and the manual yaw slider.
    auto resolve_inputs = [&](Eigen::Vector2f& tag_xy, Eigen::Matrix4f& Tob) -> bool {
        bool have_uwb = false;
        if (env_uwb) { tag_xy = {envf("UWB_X",0.f), envf("UWB_Y",0.f)}; have_uwb = true; }
        else {
            Eigen::Matrix4f s = Eigen::Matrix4f::Identity();
            if (uwb_compute_seed(uwb, sidecar, map_uwb_yaw, 0.f, s, 500)) { tag_xy = {s(0,3), s(1,3)}; have_uwb = true; }
        }
        bool have_odom = false;
        Tob = Eigen::Matrix4f::Identity();
        if (env_odom) {
            Tob = seed_from_xyyaw(envf("ODOM_X",0.f), envf("ODOM_Y",0.f), envf("ODOM_YAW_DEG",0.f)*float(M_PI)/180.f);
            Tob(2,3) = envf("ODOM_Z",0.f); have_odom = true;
        } else if (auto tf = prod.out_buf.GetData()) {
            Tob.block<3,3>(0,0) = tf->T_odom_pelvis.rotation.toRotationMatrix().cast<float>();
            Tob.block<3,1>(0,3) = tf->T_odom_pelvis.translation.cast<float>();
            have_odom = true;
        }
        return have_uwb && have_odom;
    };

    // ── MANUAL yaw slider: rotate the map by keyboard, watch the score live ────────────────────
    if (global_mode && manual_mode) {
        RawKb kb;
        float   yaw       = envf("MANUAL_YAW", 0.f);
        float   dx = 0.f, dy = 0.f;                     // manual xy nudge ON TOP of the UWB pin (m)
        bool    frozen    = false;                      // hold the UWB pin steady (kills live jitter)
        Eigen::Vector2f frozen_tag(0.f, 0.f), last_tag(0.f, 0.f);
        int64_t m_stamp   = 0;
        int     m_scans   = 0;
        bool    dirty     = true;                       // re-score/redraw needed
        std::printf("[reloc_check] MANUAL mode. yaw: j/l=-/+1, h/k=-/+10, u/o=-/+0.5 | "
                    "xy nudge: w/s=+/-x, a/d=+/-y (0.1m) | f=freeze UWB, 0=reset, space=re-score, x=quit.\n");
        auto last = std::chrono::steady_clock::now();
        while (!g_stop) {
            auto cloud = rx.cloud_buf.GetData();
            if (cloud && cloud->stamp_ns != m_stamp) { m_stamp = cloud->stamp_ns; reloc.add_scan(*cloud); ++m_scans; }

            for (int c; (c = kb.get()) != -1; ) {       // drain all pending keys
                switch (c) {
                    case 'j': yaw -= 1.f;  dirty = true; break;
                    case 'l': yaw += 1.f;  dirty = true; break;
                    case 'h': yaw -= 10.f; dirty = true; break;
                    case 'k': yaw += 10.f; dirty = true; break;
                    case 'u': yaw -= 0.5f; dirty = true; break;
                    case 'o': yaw += 0.5f; dirty = true; break;
                    case 'w': dx += 0.1f;  dirty = true; break;
                    case 's': dx -= 0.1f;  dirty = true; break;
                    case 'a': dy += 0.1f;  dirty = true; break;
                    case 'd': dy -= 0.1f;  dirty = true; break;
                    case 'f': frozen = !frozen; if (frozen) frozen_tag = last_tag; dirty = true; break;
                    case '0': yaw = 0.f; dx = 0.f; dy = 0.f; dirty = true; break;
                    case ' ':              dirty = true; break;
                    case 'x': case 'q': g_stop = true;   break;
                }
            }
            while (yaw >= 360.f) yaw -= 360.f;
            while (yaw < 0.f)    yaw += 360.f;

            const auto now = std::chrono::steady_clock::now();
            if ((dirty || now - last >= std::chrono::milliseconds(500)) && m_scans >= 5) {
                last = now; dirty = false;
                Eigen::Vector2f tag_xy; Eigen::Matrix4f Tob;
                if (!resolve_inputs(tag_xy, Tob)) { std::printf("\r[manual] waiting for UWB/odom...        "); std::fflush(stdout); }
                else {
                    last_tag = tag_xy;                                     // remember live pin (for freeze)
                    const Eigen::Vector2f base = frozen ? frozen_tag : tag_xy;
                    const Eigen::Vector2f tag_adj = base + Eigen::Vector2f(dx, dy);
                    Eigen::Matrix4f T; float med = -1.f;
                    const float r = reloc.score_yaw(tag_adj, tag_in_pelvis, Tob, yaw, &T, &med);
                    std::printf("\r[manual] yaw=%6.1f  tag=(%.2f,%.2f)%s nudge(%+.2f,%+.2f)  inlier=%.3f  median=%.3fm  n=%d   ",
                                yaw, tag_adj.x(), tag_adj.y(), frozen ? "[F]" : "   ", dx, dy, r, med, m_scans);
                    std::fflush(stdout);
                    if (viz) { viz_pub.publish_prior(reloc.prior_xyz()); viz_pub.publish_cloud(reloc.submap_xyz(T)); }
                    reloc.dump_overlay("reloc_overlay.pcd", T);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        std::printf("\n");
        rx.stop();
        if (live) { uwb.stop(); sr.stop(); }
        std::printf("[reloc_check] manual stopped.\n");
        return 0;
    }

    int64_t last_stamp = 0;
    int     n_scans    = 0;
    bool    inited     = !global_mode;   // classic mode is pre-seeded; global mode inits by search
    auto    last_align = std::chrono::steady_clock::now();
    while (!g_stop) {
        auto cloud = rx.cloud_buf.GetData();
        if (cloud && cloud->stamp_ns != last_stamp) {
            last_stamp = cloud->stamp_ns;
            reloc.add_scan(*cloud);
            ++n_scans;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - last_align >= std::chrono::milliseconds(1000)) {   // ~1 Hz
            last_align = now;
            if (!inited) {                                           // global yaw-search init
                if (n_scans < 15) {
                    std::printf("[reloc_check] accumulating submap (%d scans)...\n", n_scans);
                } else {
                    Eigen::Vector2f tag_xy_map; Eigen::Matrix4f T_odom_base;
                    if (!resolve_inputs(tag_xy_map, T_odom_base)) {
                        std::printf("[reloc_check] waiting for UWB fix / odom transform...\n");
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }
                    std::printf("[reloc_check] init inputs: tag xy=(%.2f, %.2f)  odom<-base yaw=%.1f\n",
                                tag_xy_map.x(), tag_xy_map.y(), yaw_of(T_odom_base)*57.2958f);
                    auto gr = reloc.global_init(tag_xy_map, tag_in_pelvis, T_odom_base, print_curve);
                    // Always visualize the BEST hypothesis (accept or reject) so the yaw can be
                    // eyeballed before trusting the gate: prior vs submap@best, both in map frame.
                    if (viz) {
                        viz_pub.publish_prior(reloc.prior_xyz());
                        viz_pub.publish_cloud(reloc.submap_xyz(gr.T_map_odom));
                    }
                    reloc.dump_overlay("reloc_overlay.pcd", gr.T_map_odom);
                    if (gr.ok && !no_lock) {
                        inited = true;
                        std::printf("[reloc_check] GLOBAL LOCK: yaw=%.1f inlier=%.2f conf=%.2f -> tracking.\n",
                                    gr.yaw_deg, gr.inlier_ratio, gr.confidence);
                    } else {
                        std::printf("[reloc_check] %s (best yaw=%.1f inlier=%.2f conf=%.2f) — move & watch.\n",
                                    gr.ok ? "would LOCK (NO_LOCK set)" : "not confident",
                                    gr.yaw_deg, gr.inlier_ratio, gr.confidence);
                    }
                }
            } else {                                                // tracking (GICP)
                double fit = -1.0;
                const bool ok = reloc.align(&fit);
                const Eigen::Matrix4f T = reloc.T_map_odom();
                std::printf("[reloc_check] submap=%zu prior=%zu  align=%s fitness=%.4f  "
                            "T_map_odom x=%.2f y=%.2f yaw=%.1f\n",
                            reloc.submap_size(), reloc.prior_size(), ok ? "ACCEPT" : "reject",
                            fit, T(0,3), T(1,3), yaw_of(T)*57.2958f);
                if (ok) reloc.dump_overlay("reloc_overlay.pcd");
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rx.stop();
    if (live) { uwb.stop(); sr.stop(); }
    reloc.dump_overlay("reloc_overlay.pcd");
    std::printf("[reloc_check] stopped. pcl_viewer reloc_overlay.pcd to inspect overlap.\n");
    return 0;
}
