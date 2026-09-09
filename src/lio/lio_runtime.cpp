#include "lio/lio_runtime.hpp"

#include "unitree/unitree_pointcloud_reader.hpp"
#include "unitree/unitree_pointcloud_imu_reader.hpp"

#include <algorithm>
#include <deque>
#include <iostream>

namespace kist {

namespace {
constexpr std::size_t kMaxPendingFrames = 8;    // drop oldest if LIO falls behind
constexpr std::size_t kMaxImuBuffer     = 4000; // ~20 s at 200 Hz safety cap

// Frame end time (s) = frame stamp + the largest per-point time offset (ns).
double frame_end_time(const UnitreePointCloud& f) {
    double max_off_ns = 0.0;
    if (f.has_time())
        for (float t : f.time) max_off_ns = std::max(max_off_ns, double(t));
    return f.stamp_ns * 1e-9 + max_off_ns * 1e-9;
}
}  // namespace

LioRuntime::~LioRuntime() { stop(); }

bool LioRuntime::start(int domain_id, const LioConfig& cfg,
                       const std::string& cloud_topic, const std::string& imu_topic) {
    if (running_) return true;
    worker_ = std::make_unique<LioWorker>(cfg);

    if (!UnitreePointCloudImuReader::instance().start(domain_id, "", imu_topic)) return false;
    if (!UnitreePointCloudReader::instance().start(domain_id, "", cloud_topic))  return false;

    running_ = true;
    thread_  = std::thread(&LioRuntime::loop, this);
    std::cout << "[LioRuntime] started (cloud=" << cloud_topic << " imu=" << imu_topic << ")\n";
    return true;
}

void LioRuntime::stop() {
    if (!running_) return;
    running_ = false;
    UnitreePointCloudReader::instance().stop();      // closes cloud_queue
    UnitreePointCloudImuReader::instance().stop();   // closes imu_queue -> wakes pop_all
    if (thread_.joinable()) thread_.join();
    worker_.reset();
}

void LioRuntime::loop() {
    auto& imu_rd   = UnitreePointCloudImuReader::instance();
    auto& cloud_rd = UnitreePointCloudReader::instance();

    std::deque<ImuSample>        imu_buffer;    // accumulated, not yet consumed
    std::deque<UnitreePointCloud> frames;       // pending, awaiting IMU coverage
    std::deque<ImuSample>        imu_batch;
    std::deque<UnitreePointCloud> cloud_batch;

    while (running_) {
        // Pace on the IMU queue (blocks ~5 ms at 200 Hz; false = closed -> exit).
        if (!imu_rd.imu_queue.pop_all(imu_batch)) break;
        for (auto& s : imu_batch) imu_buffer.push_back(s);
        while (imu_buffer.size() > kMaxImuBuffer) imu_buffer.pop_front();

        // Drain clouds opportunistically.
        if (cloud_rd.cloud_queue.try_pop_all(cloud_batch))
            for (auto& c : cloud_batch) frames.push_back(std::move(c));
        while (frames.size() > kMaxPendingFrames) {
            frames.pop_front();
            frames_dropped.fetch_add(1, std::memory_order_relaxed);
        }

        // Process every frame whose end time is now covered by buffered IMU.
        while (!frames.empty()) {
            const UnitreePointCloud& frame = frames.front();
            const double end_t = frame_end_time(frame);

            if (imu_buffer.empty() || imu_buffer.back().stamp_ns * 1e-9 < end_t)
                break;   // wait for more IMU

            // Collect the IMU window [.. end_t] and consume it from the buffer;
            // samples past end_t stay for the next frame (the worker's own last_imu_
            // bridges the gap).
            std::deque<ImuSample> window;
            while (!imu_buffer.empty() && imu_buffer.front().stamp_ns * 1e-9 <= end_t) {
                window.push_back(imu_buffer.front());
                imu_buffer.pop_front();
            }

            const LioPose pose = worker_->process(frame, window);
            frames_processed.fetch_add(1, std::memory_order_relaxed);
            if (pose.valid) pose_buf.SetData(pose);
            frames.pop_front();
        }
    }
    std::cout << "[LioRuntime] loop exited (" << frames_processed.load()
              << " frames processed)\n";
}

} // namespace kist
