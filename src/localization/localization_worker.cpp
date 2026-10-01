#include "localization/localization_worker.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>

namespace kist {

namespace {
// Map the console phase string to a LocSample::Phase code (kept next to the strings they mirror).
int phase_code(const char* p) {
    if (std::strcmp(p, "track") == 0)              return LocSample::Track;
    if (std::strcmp(p, "LOCK(init)") == 0)         return LocSample::LockInit;
    if (std::strcmp(p, "LOCK(fixed-seed)") == 0)   return LocSample::LockFixedSeed;
    if (std::strcmp(p, "track(uwb-spike)") == 0)   return LocSample::UwbSpike;
    if (std::strcmp(p, "LOST->reinit") == 0)       return LocSample::LostReinit;
    return LocSample::Search;
}
}  // namespace

bool LocalizationWorker::start(LioReceiver& rx, LioTransformProducer& prod, UwbReceiver* uwb,
                               const std::string& prior_pcd, const std::string& sidecar,
                               float map_uwb_yaw_rad, const Eigen::Vector3f& tag_in_pelvis,
                               const Eigen::Matrix4f& fallback_seed, DataBuffer<MapOdom>& out,
                               const RelocConfig& rcfg, const LocFilterConfig& fcfg,
                               DataBuffer<LocSample>* sample_out) {
    reloc_.set_config(rcfg);                       // before load_prior (uses map_voxel_m)
    if (!reloc_.load_prior(prior_pcd)) {
        std::cerr << "[LocalizationWorker] prior map load failed: " << prior_pcd << "\n";
        return false;
    }
    ekf_.set_config(fcfg);
    rx_ = &rx; prod_ = &prod; uwb_ = uwb; out_ = &out; sample_out_ = sample_out;
    sidecar_ = sidecar; map_uwb_yaw_ = map_uwb_yaw_rad; tag_in_pelvis_ = tag_in_pelvis;
    fallback_seed_ = fallback_seed;
    running_ = true;
    thread_ = std::thread(&LocalizationWorker::run, this);
    return true;
}

void LocalizationWorker::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

bool LocalizationWorker::resolve_inputs(Eigen::Vector2f& uwb_xy, bool& have_uwb,
                                        Eigen::Matrix4f& Tob) const {
    have_uwb = false;
    if (uwb_) {
        Eigen::Matrix4f s = Eigen::Matrix4f::Identity();
        if (uwb_compute_seed(*uwb_, sidecar_, map_uwb_yaw_, 0.f, s, 200)) {
            uwb_xy = {s(0, 3), s(1, 3)};               // robot xy in map (double transform)
            have_uwb = true;
        }
    }
    if (auto tf = prod_->out_buf.GetData()) {
        Tob = Eigen::Matrix4f::Identity();
        Tob.block<3,3>(0,0) = tf->T_odom_pelvis.rotation.toRotationMatrix().cast<float>();
        Tob.block<3,1>(0,3) = tf->T_odom_pelvis.translation.cast<float>();
        return true;                                    // have_odom
    }
    return false;
}

void LocalizationWorker::run() {
    int64_t last_stamp = 0;
    int     n_scans    = 0;
    int     lost_count = 0;                       // consecutive UWB-LOST frames (sustained -> reinit)
    constexpr int kLostFrames = 5;                // re-init only after this many consecutive LOST
    auto    last_step  = std::chrono::steady_clock::now();
    std::string last_phase_printed;               // console prints only on a phase change
    while (running_) {
        auto cloud = rx_->cloud_buf.GetData();
        if (cloud && cloud->stamp_ns != last_stamp) {
            last_stamp = cloud->stamp_ns;
            reloc_.add_scan(*cloud);
            ++n_scans;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_step >= std::chrono::milliseconds(1000)) {   // ~1 Hz
            const double dt = std::chrono::duration<double>(now - last_step).count();  // actual elapsed
            last_step = now;

            Eigen::Vector2f uwb_xy; bool have_uwb = false; Eigen::Matrix4f Tob;
            const bool have_odom = resolve_inputs(uwb_xy, have_uwb, Tob);

            const char* phase = "search";
            double gicp_fit = -1.0;               // captured for the diagnostics snapshot
            if (!ekf_.seeded()) {
                // ── GLOBAL-INIT: yaw-search -> seed the EKF on a confident/stable lock ──
                if (n_scans >= 15 && have_odom && have_uwb) {
                    auto gr = reloc_.global_init(uwb_xy, tag_in_pelvis_, Tob);
                    if (gr.ok) { ekf_.seed(gr.T_map_odom); phase = "LOCK(init)"; }
                } else if (n_scans >= 15 && !uwb_) {
                    ekf_.seed(fallback_seed_);            // no UWB configured -> fixed seed fallback
                    phase = "LOCK(fixed-seed)";
                }
            } else {
                // ── TRACKING: EKF fuses GICP + UWB ──
                phase = "track";
                ekf_.predict(dt);
                Eigen::Matrix4f T_gicp;
                if (reloc_.gicp_measure(ekf_.T_map_odom(), &T_gicp, &gicp_fit)) ekf_.update_gicp(T_gicp);
                if (have_uwb && have_odom) {
                    bool lost = false;
                    ekf_.update_uwb(uwb_xy.cast<double>(), Tob, tag_in_pelvis_.cast<double>(), &lost);
                    // Reset only on SUSTAINED disagreement — a single UWB jitter spike is ignored
                    // (its update was already gated out; one spike must not drop the lock).
                    if (lost) {
                        if (++lost_count >= kLostFrames) { ekf_.reset(); lost_count = 0; phase = "LOST->reinit"; }
                        else phase = "track(uwb-spike)";
                    } else {
                        lost_count = 0;
                    }
                }
                if (ekf_.seeded()) out_->SetData(MapOdom{ekf_.T_map_odom()});
            }

            // Console: only on a phase change (LOCK / LOST / uwb-spike events). Per-cycle numbers
            // (mapodom, gicp fitness, uwb) live in the trace.
            if (last_phase_printed != phase) {
                last_phase_printed = phase;
                const Eigen::Vector3d s = ekf_.state();
                std::printf("[localization] %s submap=%zu  T_map_odom x=%.2f y=%.2f yaw=%.1f\n",
                            phase, reloc_.submap_size(), s(0), s(1), s(2) * 57.2958);
            }

            if (sample_out_) {                    // diagnostics snapshot for NavTrace
                const Eigen::Vector3d s = ekf_.state();
                LocSample ls;
                ls.phase        = phase_code(phase);
                ls.mapodom_x    = float(s(0)); ls.mapodom_y = float(s(1)); ls.mapodom_yaw = float(s(2));
                ls.gicp_fitness = float(gicp_fit);
                ls.submap_pts   = int(reloc_.submap_size());
                ls.have_uwb     = have_uwb ? 1 : 0;
                if (have_uwb) { ls.uwb_map_x = uwb_xy.x(); ls.uwb_map_y = uwb_xy.y(); }
                sample_out_->SetData(ls);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

} // namespace kist
