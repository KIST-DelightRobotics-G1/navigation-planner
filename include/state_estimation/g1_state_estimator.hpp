#pragma once

// G1 odom->pelvis estimator: a thin facade over a contact-aided InEKF (RossHartley
// invariant-ekf). The filter type is fully hidden (PIMPL) so NO inekf symbol leaks
// into the header — deskew / transform tree / planner include this and never see
// invariant-ekf, so the backend can be swapped without touching them.
//
//   ImuSample / LegJointSample / FootContactSample  ->  G1StateEstimator  ->  EstimatorState
//
// The estimate is T_odom_pelvis(t) (+ velocity, biases). A producer bridges the
// pose into the transform tree; that wiring lives elsewhere (not here).

#include "state_estimation/estimator_state.hpp"
#include "state_estimation/estimator_input.hpp"

#include <memory>

namespace kist {

struct EstimatorConfig {
    // InEKF noise std-devs — upstream example defaults; retune on the G1.
    double gyro_noise           = 0.01;
    double accel_noise          = 0.1;
    double gyro_bias_noise      = 0.00001;
    double accel_bias_noise     = 0.0001;
    double contact_noise        = 0.01;
    // Foot-FK kinematic-measurement covariance (diagonal): rotation then position.
    double fk_rotation_std      = 0.01;   // rad
    double fk_position_std      = 0.01;   // m
    // Skip a propagation whose dt is non-positive or larger than this (dropout).
    double max_propagate_dt_s   = 0.05;
};

class G1StateEstimator {
public:
    explicit G1StateEstimator(const EstimatorConfig& cfg = {});
    ~G1StateEstimator();

    G1StateEstimator(const G1StateEstimator&) = delete;
    G1StateEstimator& operator=(const G1StateEstimator&) = delete;

    // (Re)initialize the pelvis-in-odom state (pose + velocity + biases).
    void reset(const EstimatorState& initial);

    // High-rate IMU propagation. Integrates the PREVIOUS sample over dt; the first
    // call only seeds the timestamp (no dt yet).
    void propagate(const ImuSample& imu);

    // Contact-aided kinematic correction: a contacted foot is (near) stationary in
    // the world, so its FK-measured pose (our G1Kinematics leg FK) corrects drift.
    void updateKinematics(const LegJointSample& joints, const FootContactSample& contacts);

    // Latest estimate (cached each step, so this is const-safe even though the
    // underlying library getters are not const).
    EstimatorState state() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;   // holds inekf::InEKF — no InEKF type in this header
};

} // namespace kist
