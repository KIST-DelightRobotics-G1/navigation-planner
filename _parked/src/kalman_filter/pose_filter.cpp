#include "kalman_filter/pose_filter.hpp"

#include <yaml-cpp/yaml.h>

#include <pthread.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <system_error>
#include <utility>

namespace kist {

namespace {

// orientation quaternion -> yaw (rad, CCW positive) — z-axis rotation.
double quat_to_yaw(float qx, float qy, float qz, float qw) {
    return std::atan2(2.0 * (double(qw) * qz + double(qx) * qy),
                      1.0 - 2.0 * (double(qy) * qy + double(qz) * qz));
}

} // namespace

PoseFilterOptions pose_filter_options_from_yaml(const YAML::Node& section) {
    PoseFilterOptions o;
    if (!section) return o;  // absent section -> all defaults
    o.frame_id       = section["frame_id"].as<std::string>(o.frame_id);
    o.uwb_timeout_ms = section["uwb_timeout_ms"].as<double>(o.uwb_timeout_ms);
    o.yaw_init       = section["yaw_init"].as<bool>(o.yaw_init);
    o.yaw_state_file = section["yaw_state_file"].as<std::string>(o.yaw_state_file);
    if (const auto y = section["initial_yaw_deg"]) {   // optional manual start heading
        o.has_initial_yaw = true;
        o.initial_yaw_rad = y.as<double>(0.0) * M_PI / 180.0;
    }
    o.ekf            = uwb_odom_aekf_params_from_yaml(section);
    return o;
}

PoseFilter::PoseFilter(PoseFilterOptions options)
    : ekf_(options.ekf),
      frame_id_(std::move(options.frame_id)),
      uwb_timeout_ms_(options.uwb_timeout_ms),
      yaw_state_file_(std::move(options.yaw_state_file)) {
    // Seed uncertainty: half the calibrated-threshold std -> comfortably below
    // the gate, so a reused bias reads as calibrated immediately.
    const double seed_std = 0.5 * options.ekf.bias_calibrated_std_deg * M_PI / 180.0;
    seed_var_ = seed_std * seed_std;

    if (options.has_initial_yaw) {
        // Manual start heading wins: seed the WORLD yaw directly at init.
        have_manual_yaw_ = true;
        manual_yaw_rad_  = options.initial_yaw_rad;
        std::cerr << "[PoseFilter] initial yaw set to "
                  << (manual_yaw_rad_ * 180.0 / M_PI) << " deg (manual, skips calibration)\n";
    } else if (!options.yaw_init && !yaw_state_file_.empty()) {
        std::ifstream in(yaw_state_file_);
        if (double b; in >> b) {
            seed_b_theta_ = b;
            have_seed_    = true;
            std::cerr << "[PoseFilter] reusing saved yaw bias "
                      << (b * 180.0 / M_PI) << " deg from " << yaw_state_file_ << "\n";
        } else {
            std::cerr << "[PoseFilter] yaw_init=false but no saved bias at "
                      << yaw_state_file_ << " - will calibrate from motion\n";
        }
    }
}

PoseFilter::~PoseFilter() { stop(); }

bool PoseFilter::start(DataBuffer<UnitreeOdometry>& odom_src) {
    if (running_) return true;
    odom_src_ = &odom_src;
    running_  = true;
    thread_   = std::thread(&PoseFilter::run, this);
    return true;
}

void PoseFilter::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();

    // Persist the calibrated yaw bias for a future yaw_init=false run. Saved
    // regardless of yaw_init so there is always a fresh value to reuse next time.
    if (!yaw_state_file_.empty() && ekf_.initialized() && ekf_.yaw_calibrated()) {
        std::error_code ec;
        const std::filesystem::path path(yaw_state_file_);
        if (path.has_parent_path())
            std::filesystem::create_directories(path.parent_path(), ec);
        if (std::ofstream out(yaw_state_file_, std::ios::trunc); out) {
            out << ekf_.b_theta_rad() << "\n";
            std::cerr << "[PoseFilter] saved yaw bias "
                      << (ekf_.b_theta_rad() * 180.0 / M_PI) << " deg to "
                      << yaw_state_file_ << "\n";
        }
    }
}

// ── gates ─────────────────────────────────────────────────────────────────

PoseFilter::OdomHealth
PoseFilter::odom_gate(std::shared_ptr<const UnitreeOdometry>& sample) {
    auto odom = odom_src_->GetData();
    if (!odom) {                            // reader watchdog cleared it -> dead
        odom_gap_ = true;
        return OdomHealth::Lost;
    }
    if (odom->stamp_ns == last_odom_stamp_)
        return OdomHealth::Idle;            // alive, no new sample this poll

    if (odom_gap_) {                        // resumed after a gap
        odom_gap_ = false;
        ekf_.reset_odom_reference();        // skip the cross-gap increment
    }
    last_odom_stamp_ = odom->stamp_ns;
    sample = std::move(odom);
    return OdomHealth::Fresh;
}

bool PoseFilter::uwb_gate(double odom_yaw) {
    if (auto fix = uwb_buf.GetData(); fix && fix->stamp_ns != last_uwb_stamp_) {
        last_uwb_stamp_   = fix->stamp_ns;
        last_uwb_advance_ = std::chrono::steady_clock::now();
        if (!ekf_.initialized()) {
            if (have_manual_yaw_) {              // config-set world heading: b = yaw - odom_yaw
                ekf_.initialize(fix->x, fix->y, odom_yaw, manual_yaw_rad_ - odom_yaw, seed_var_);
                have_manual_yaw_ = false;
            } else if (have_seed_) {             // reuse the saved yaw bias
                ekf_.initialize(fix->x, fix->y, odom_yaw, seed_b_theta_, seed_var_);
                have_seed_ = false;
            } else {
                ekf_.initialize(fix->x, fix->y, odom_yaw);
            }
        } else {
            ekf_.update(fix->x, fix->y);
        }
    }

    if (uwb_timeout_ms_ <= 0.0) return true;                     // gate disabled
    if (last_uwb_advance_.time_since_epoch().count() == 0)       // no fix ever
        return false;
    const auto age = std::chrono::steady_clock::now() - last_uwb_advance_;
    return std::chrono::duration<double, std::milli>(age).count() <= uwb_timeout_ms_;
}

// ── output ──────────────────────────────────────────────────────────────────

void PoseFilter::emit(bool trustworthy,
                      const std::shared_ptr<const UnitreeOdometry>& sample,
                      const char* reason) {
    if (trustworthy) {
        CalibratedPose p;
        p.stamp_ns = sample->stamp_ns;
        p.x        = float(ekf_.x_m());
        p.y        = float(ekf_.y_m());
        p.yaw      = float(ekf_.global_yaw_rad());
        p.frame_id = frame_id_;
        calibrated_pose_buf.SetData(std::move(p));
        if (!emitting_) {
            emitting_ = true;
            std::cerr << "[PoseFilter] pose stream up\n";
        }
    } else {
        calibrated_pose_buf.Clear();
        if (emitting_) {
            emitting_ = false;
            std::cerr << "[PoseFilter] pose gated - " << reason << "\n";
        }
    }
}

// ── worker ──────────────────────────────────────────────────────────────────

void PoseFilter::run() {
    pthread_setname_np(pthread_self(), "pose-filter");
    using namespace std::chrono_literals;

    while (running_) {
        std::shared_ptr<const UnitreeOdometry> odom;
        switch (odom_gate(odom)) {
            case OdomHealth::Lost:
                emit(false, nullptr, "odom lost");
                std::this_thread::sleep_for(2ms);
                continue;
            case OdomHealth::Idle:
                std::this_thread::sleep_for(2ms);
                continue;
            case OdomHealth::Fresh:
                break;
        }

        const double yaw = quat_to_yaw(odom->qx, odom->qy, odom->qz, odom->qw);
        const bool uwb_fresh = uwb_gate(yaw);
        ekf_.predict(odom->px, odom->py, yaw);

        if (uwb_fresh && ekf_.yaw_calibrated())
            emit(true, odom, nullptr);
        else
            emit(false, odom, uwb_fresh ? "calibrating" : "uwb stale");
    }
}

} // namespace kist
