#pragma once

// LioTransformProducer — composes the LIO sensor pose + the waist FK into the stable
// robot transforms and publishes them to a thread-safe buffer, so consumers read a
// buffer like every other reader (no tree / mutex exposed).
//
//   T_odom_lidar  = T_odom_lidar_imu (LIO) * lidar_imu->lidar (ext)
//   T_odom_pelvis = T_odom_lidar * (torso->lidar)^-1 * (pelvis->torso : waist FK)^-1
//
// step() reads the latest LIO pose + the latest lowstate (waist joints) and recomposes.
// A fixed chain evaluated at latest-time, so a plain compose (not a shared tree) suffices;
// the DataBuffer is the thread boundary. Call step() from wherever the LIO frame lands
// (the LIO receive thread, or a runtime loop) — the producer keeps no thread of its own.

#include "common/data_buffer.hpp"
#include "common/stamp_ring.hpp"
#include "lio/lio_odometry.hpp"
#include "lio/robot_transforms.hpp"
#include "transforms/transform.hpp"
#include "unitree/unitree_state.hpp"

#include <cstdint>
#include <optional>

namespace kist {

class LioTransformProducer {
public:
    LioTransformProducer();

    // Compose from the latest LIO pose + lowstate, publish to out_buf (latest) and
    // into the stamp ring (for stamp-matched lookup).
    void step(const LioOdometry& odom, const UnitreeState& state);

    DataBuffer<RobotTransforms> out_buf;   // latest transforms (for latest-value consumers)

    // The transforms whose stamp is closest to stamp_ns, within max_dt_ns. The mapping
    // stage uses this to pair a specific registered scan with the pose (ray origin +
    // floor ref) that belongs to it, rather than an arbitrary neighbouring LIO cycle.
    std::optional<RobotTransforms> nearest(int64_t stamp_ns,
                                           int64_t max_dt_ns = 100'000'000) const {
        return hist_.nearest(stamp_ns, max_dt_ns);
    }

private:
    Transform T_lidar_imu_lidar_;  // Mid-360 imu -> lidar scan centre (datasheet ext)
    Transform T_torso_lidar_;      // torso -> lidar mount (URDF; CALIBRATION — see .cpp)

    StampRing<RobotTransforms, 32> hist_;   // recent transforms, stamp-indexed
};

} // namespace kist
