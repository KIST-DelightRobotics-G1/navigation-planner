#pragma once

// The LIO runtime: the actual wiring from the robot's live DDS streams to
// T_odom_lidar. It starts the Mid-360 cloud + IMU readers, runs one worker thread
// that drains both queues, pairs each frame with the IMU samples spanning it (the
// role of FAST-LIO's sync_packages), calls the LioWorker, and publishes the pose.
//
//   cloud reader (DDS) -> cloud_queue ─┐
//   imu reader   (DDS) -> imu_queue   ─┤→ [sync thread] frame + IMU window
//                                        → LioWorker.process → LioResult → result_buf
//
// The runtime paces on the IMU queue (200 Hz) and drains the cloud queue
// non-blocking, so a frame is processed only once IMU covers its end time.

#include "common/data_buffer.hpp"
#include "lio/lio_worker.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace kist {

class LioRuntime {
public:
    LioRuntime() = default;
    ~LioRuntime();

    // Start the Mid-360 readers on the given DDS domain (apply_dds_config() must
    // already have run) and spawn the LIO thread. Optional topic overrides.
    bool start(int domain_id, const LioConfig& cfg,
               const std::string& cloud_topic = "rt/utlidar/cloud_livox_mid360",
               const std::string& imu_topic   = "rt/utlidar/imu_livox_mid360");
    void stop();

    // Latest LIO output: pose (T_odom_lidar) + registered cloud in odom. Empty until
    // the filter has produced a valid result.
    DataBuffer<LioResult> result_buf;

    std::atomic<uint64_t> frames_processed{0};
    std::atomic<uint64_t> frames_dropped{0};      // dropped when LIO falls behind

private:
    void loop();

    std::unique_ptr<LioWorker> worker_;
    std::thread                thread_;
    std::atomic<bool>          running_{false};
};

} // namespace kist
