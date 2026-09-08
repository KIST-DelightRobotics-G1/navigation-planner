#pragma once

// Worker thread that turns the latest depth frame + instance masks into a labeled
// point cloud in the ROBOT frame, and publishes it on its own buffer:
//   build_labeled_cloud  (deproject depth + tag each point with its class)
//   transform_cloud      (camera -> robot base, via the mount extrinsics)
// Inputs are two DataBuffers (depth, mask) injected at start() — the same
// KF/reader style used across this repo; the compute is the free functions in
// labeled_cloud.hpp + camera_extrinsics.hpp. The mount extrinsics can be retuned
// at runtime via set_extrinsics() (for the live-tuning viewer). One instance per
// camera. A consumer only start()/stop()s it and reads result().

#include "common/data_buffer.hpp"
#include "labeled_cloud/labeled_cloud.hpp"       // LabeledCloud
#include "labeled_cloud/camera_extrinsics.hpp"   // CameraExtrinsics
#include "labeled_cloud/ground_plane.hpp"        // GroundPlane(Config)
#include "realsense/depth_frame.hpp"                          // DepthFrame (ext)
#include "cv/yolo/instance_segmentation/yolo_inst_seg_frame.hpp"  // InstSegFrame

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>

namespace kist {

struct LabeledCloudConfig {
    CameraExtrinsics  extrinsics;   // camera optical -> robot base
    int               stride = 2;   // depth subsample (1 = every pixel)
    GroundPlaneConfig ground;       // per-frame floor fit -> tilt leveling
};

class LabeledCloudGenerator {
public:
    LabeledCloudGenerator() = default;
    ~LabeledCloudGenerator() { stop(); }

    LabeledCloudGenerator(const LabeledCloudGenerator&) = delete;
    LabeledCloudGenerator& operator=(const LabeledCloudGenerator&) = delete;

    // Start the worker off the depth + mask buffers. False if already running.
    bool start(DataBuffer<DepthFrame>&  depth_src,
               DataBuffer<InstSegFrame>& mask_src,
               const LabeledCloudConfig& cfg);
    void stop();

    bool running() const { return running_; }

    // Retune the camera mount live (thread-safe; applied on the next cycle).
    void set_extrinsics(const CameraExtrinsics& e) {
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        extrinsics_ = e;
    }

    // ── output / telemetry ──
    DataBuffer<LabeledCloud>& result() { return cloud_buf_; }
    uint64_t frames_processed() const { return processed_.load(std::memory_order_relaxed); }

private:
    void run();

    DataBuffer<DepthFrame>*   depth_src_ = nullptr;
    DataBuffer<InstSegFrame>*  mask_src_  = nullptr;
    DataBuffer<LabeledCloud>  cloud_buf_;

    std::mutex       cfg_mtx_;
    CameraExtrinsics extrinsics_;
    int              stride_ = 2;
    GroundPlaneConfig ground_cfg_;
    GroundPlane       prev_ground_;   // temporal prior for the floor fit

    std::thread           thread_;
    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> processed_{0};
};

} // namespace kist
