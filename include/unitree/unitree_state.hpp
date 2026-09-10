#pragma once

// The G1's full low-level state — one decoded LowState packet. The reader hands this
// out whole and consumers parse what they need (waist joints 12/13/14 for the waist FK,
// leg joints for foot FK, dq for a safety check, ...). Same shape as
// kist-gearsonic-inference's unitree_state.hpp, plus a receive stamp for time-sync.

#include "unitree/imu.hpp"

#include <array>
#include <cstdint>

namespace kist {

constexpr int kNumMotors = 29;  // Unitree G1

struct MotorState {
    double q{0.0};    // position (rad)
    double dq{0.0};   // velocity (rad/s)
    double tau{0.0};  // estimated torque (Nm)
};

struct UnitreeState {
    int64_t  stamp_ns{0};     // receive time (LowState carries no header stamp; for time-sync)
    std::array<MotorState, kNumMotors> motors{};
    IMU      imu_pelvis{};     // pelvis IMU (from LowState.imu_state)
    uint32_t tick{0};          // robot's own ms counter
    uint8_t  mode_machine{0};  // robot variant code
};

} // namespace kist
