#pragma once

#include "common/data_buffer.hpp"
#include "kalman_filter/calibrated_pose.hpp"
#include "kalman_filter/uwb_fix.hpp"
#include "kalman_filter/uwb_odom_aekf.hpp"
#include "unitree/unitree_odometry.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace YAML {
class Node;
}

namespace kist {

// Tuning + policy for PoseFilter (the `pose_filter` config section). Bundles
// the output frame, the UWB-staleness safety timeout, and the EKF params.
struct PoseFilterOptions {
    std::string       frame_id = "map";
    // UWB staleness timeout (ms): if no fresh fix arrives within this window
    // the pose is no longer UWB-anchored, so output is gated off. 0 disables
    // the gate. Default matches ext-sensor-io's UwbSubscriber watchdog cadence
    // (10 missed fixes at 10Hz).
    double            uwb_timeout_ms = 1000.0;

    // Yaw-bias persistence. The calibrated bias is saved to yaw_state_file on
    // stop(). yaw_init=true (default) ignores any saved bias and recalibrates
    // from motion every run. yaw_init=false reuses the saved bias to skip the
    // drive-to-calibrate step — only valid when the robot's odom is continuous
    // across the restart (no reboot, and it didn't rotate while the process was
    // down). Empty yaw_state_file disables persistence.
    bool              yaw_init = true;
    std::string       yaw_state_file;

    // Manual initial heading. When set (config key initial_yaw_deg present), the
    // EKF is seeded so the robot's WORLD heading is this value at startup — no
    // drive-to-calibrate, no saved file. Takes priority over yaw_init/state_file.
    // Valid only if the robot actually starts at that heading (world frame:
    // +x = 0 deg, +y = 90 deg, CCW). rad internally.
    bool              has_initial_yaw = false;
    double            initial_yaw_rad = 0.0;

    UwbOdomAEKFParams ekf;
};

// Reads the `pose_filter` section of config.yaml (pass
// Config::instance().root()["pose_filter"]). Missing keys keep their
// defaults, so a partial/absent section is fine.
PoseFilterOptions pose_filter_options_from_yaml(const YAML::Node& section);

// UWB + odometry fusion filter — owns the EKF on its own worker thread and
// turns the two input streams into a global-pose stream. Thread + Buffer:
// odom is pulled from an injected DataBuffer (the odom reader's odom_buf),
// UWB fixes are pushed into uwb_buf by the wiring layer, and the fused pose
// is published to calibrated_pose_buf. The EKF is touched only by this one
// thread, so it needs no locking; the DataBuffers handle every cross-thread
// boundary.
//
// Wiring (at the assembly point):
//   PoseFilter filter(opts);
//   filter.start(UnitreeOdometryReader::instance().odom_buf);
//   uwbReceiver.set_on_position(
//       [&](const UwbPosition& p){ filter.uwb_buf.SetData({p.stamp_ns, p.x, p.y}); });
//
// Safety (both degrade the OUTPUT, never the process — an empty buffer means
// "no trusted pose", and both recover automatically when the source returns):
//   * odom_gate — odom dropout clears the pose and re-seeds the EKF on resume
//                 (no jump across the gap). Always on.
//   * uwb_gate  — UWB going stale (no fix within uwb_timeout_ms) gates the pose
//                 off; short between-fix gaps are normal EKF coasting.
class PoseFilter {
public:
    explicit PoseFilter(PoseFilterOptions options = {});
    ~PoseFilter();

    bool start(DataBuffer<UnitreeOdometry>& odom_src);
    void stop();

    // ── input: latest UWB fix (pushed by the wiring callback) ──
    DataBuffer<UwbFix> uwb_buf;

    // ── output: fused global pose (empty until trusted; see the gates) ──
    DataBuffer<CalibratedPose> calibrated_pose_buf;

private:
    // odom stream state this tick.
    enum class OdomHealth { Lost, Idle, Fresh };

    void run();

    // Odom gate: fetch the latest odom and classify it. On a fresh, continuous
    // sample returns Fresh with `sample` set; re-seeds the EKF when odom
    // resumes after a gap. Touches only the EKF baseline, never the output.
    OdomHealth odom_gate(std::shared_ptr<const UnitreeOdometry>& sample);

    // UWB gate: fold a new fix into the EKF (initialize/update) and refresh the
    // staleness clock; returns whether UWB is currently fresh (within timeout).
    bool uwb_gate(double odom_yaw);

    // Single point that mutates the output buffer + logs on state changes.
    // trustworthy=true publishes (needs `sample`); false clears with `reason`.
    void emit(bool trustworthy,
              const std::shared_ptr<const UnitreeOdometry>& sample,
              const char* reason);

    UwbOdomAEKF ekf_;
    std::string frame_id_;
    double      uwb_timeout_ms_;

    // Yaw-bias persistence (see PoseFilterOptions).
    std::string yaw_state_file_;
    bool        have_seed_ = false;   // a saved bias is queued for the next init
    double      seed_b_theta_ = 0.0;  // rad
    double      seed_var_ = 0.0;      // rad^2 — small -> immediately calibrated
    bool        have_manual_yaw_ = false;  // a config-set initial world heading is queued
    double      manual_yaw_rad_ = 0.0;     // desired world heading at init (rad)

    DataBuffer<UnitreeOdometry>* odom_src_ = nullptr;
    int64_t last_odom_stamp_ = -1;
    int64_t last_uwb_stamp_  = -1;
    bool    odom_gap_ = false;                                    // odom currently lost
    std::chrono::steady_clock::time_point last_uwb_advance_{};    // last new-fix time
    bool    emitting_ = false;                                    // for edge-triggered logs

    std::thread       thread_;
    std::atomic<bool> running_{false};
};

} // namespace kist
