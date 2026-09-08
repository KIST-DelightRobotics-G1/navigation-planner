#include "unitree/unitree_pointcloud_reader.hpp"

#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

namespace kist {

// ─── PointCloud2 (ROS2 IDL) → UnitreePointCloud ──────────────────────────────
// PointCloud2 has no CRC, so validation is structural: the FLOAT32 x/y/z fields
// must exist (looked up by name — offsets are not assumed). The optional Mid-360
// per-point channels (intensity FLOAT32, ring UINT16, time FLOAT32) are captured
// when present, in lockstep with xyz. Walks the blob by point_step, drops
// non-finite points. Returns false when x/y/z are missing (frame_out untouched).

namespace {

constexpr uint8_t kPointFieldUint16  = 4;  // sensor_msgs/PointField datatypes
constexpr uint8_t kPointFieldFloat32 = 7;

struct FieldRef {
    uint32_t offset = 0;
    bool     found  = false;
};

FieldRef find_field(const std::vector<sensor_msgs::msg::dds_::PointField_>& fields,
                    const char* name, uint8_t datatype) {
    for (const auto& f : fields)
        if (f.datatype() == datatype && f.name() == name)
            return {f.offset(), true};
    return {};
}

bool decode_pointcloud2(const sensor_msgs::msg::dds_::PointCloud2_& msg,
                        UnitreePointCloud& frame_out) {
    const auto& fields = msg.fields();
    const FieldRef fx = find_field(fields, "x", kPointFieldFloat32);
    const FieldRef fy = find_field(fields, "y", kPointFieldFloat32);
    const FieldRef fz = find_field(fields, "z", kPointFieldFloat32);
    if (!(fx.found && fy.found && fz.found))
        return false;

    const FieldRef fi = find_field(fields, "intensity", kPointFieldFloat32);
    const FieldRef fr = find_field(fields, "ring",      kPointFieldUint16);
    const FieldRef ft = find_field(fields, "time",      kPointFieldFloat32);

    frame_out.stamp_ns = int64_t(msg.header().stamp().sec()) * 1000000000LL +
                         msg.header().stamp().nanosec();
    frame_out.frame_id = msg.header().frame_id();

    const std::size_t n_points = std::size_t(msg.width()) * msg.height();
    const std::size_t stride   = msg.point_step();
    const uint8_t*    blob     = msg.data().data();

    frame_out.xyz.clear();       frame_out.xyz.reserve(n_points * 3);
    frame_out.intensity.clear(); if (fi.found) frame_out.intensity.reserve(n_points);
    frame_out.ring.clear();      if (fr.found) frame_out.ring.reserve(n_points);
    frame_out.time.clear();      if (ft.found) frame_out.time.reserve(n_points);

    for (std::size_t i = 0; i < n_points; ++i) {
        const uint8_t* rec = blob + i * stride;
        float x, y, z;
        std::memcpy(&x, rec + fx.offset, sizeof(float));
        std::memcpy(&y, rec + fy.offset, sizeof(float));
        std::memcpy(&z, rec + fz.offset, sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;
        frame_out.xyz.insert(frame_out.xyz.end(), {x, y, z});
        if (fi.found) { float v;    std::memcpy(&v, rec + fi.offset, sizeof(v)); frame_out.intensity.push_back(v); }
        if (fr.found) { uint16_t v; std::memcpy(&v, rec + fr.offset, sizeof(v)); frame_out.ring.push_back(v); }
        if (ft.found) { float v;    std::memcpy(&v, rec + ft.offset, sizeof(v)); frame_out.time.push_back(v); }
    }
    return true;
}

} // namespace

// ─── UnitreePointCloudReader ──────────────────────────────────────────────────

UnitreePointCloudReader& UnitreePointCloudReader::instance() {
    static UnitreePointCloudReader inst;
    return inst;
}

bool UnitreePointCloudReader::start(int domain_id, const std::string& network_interface,
                                    const std::string& topic, std::size_t queue_capacity) {
    cloud_queue.open(queue_capacity);
    try {
        // Safe when the embedding process already initialized the factory
        // (Init is a no-op after the first call in the same process).
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

        cloud_sub_.reset(new CloudSub(topic));
        cloud_sub_->InitChannel(
            [this](const void* msg) { on_cloud_update(msg); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[UnitreePointCloudReader] DDS init failed on interface \""
                  << network_interface << "\": " << e.what()
                  << "\n  Check the robot LAN cable.\n";
        return false;
    }

    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&UnitreePointCloudReader::watchdog_loop, this);
    std::cout << "[UnitreePointCloudReader] started on domain=" << domain_id
              << " interface=" << network_interface
              << " topic=" << topic << "\n";
    return true;
}

void UnitreePointCloudReader::stop() {
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    cloud_queue.close();     // wake/drain the LIO consumer
    cloud_sub_.reset();
}

void UnitreePointCloudReader::set_processor(ProcessFn fn) {
    process_ = std::move(fn);
}

void UnitreePointCloudReader::on_cloud_update(const void* message) {
    const auto& msg = *static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(message);

    UnitreePointCloud frame;
    if (!decode_pointcloud2(msg, frame)) {
        std::cerr << "[UnitreePointCloudReader] cloud without FLOAT32 x/y/z fields - dropped\n";
        return;
    }
    if (process_)
        process_(frame);
    cloud_buf.SetData(frame);                          // latest snapshot (+ watchdog)
    if (!cloud_queue.push(std::move(frame)))           // LIO hand-off (every frame)
        dropped.fetch_add(1, std::memory_order_relaxed);
}

// LiDAR streams at ~10Hz; 500ms (5 frames) of silence clears the buffer
// so downstream mapping stops extending a stale world. Polled at 10ms —
// same cadence as gearsonic's UnitreeStateReader watchdog.
void UnitreePointCloudReader::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 500.0;  // 5 frames at 10Hz

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(10ms);

        auto cloud = cloud_buf.GetDataWithTime();
        if (cloud.HasData() && cloud.GetAgeMs() > stale_ms) {
            std::cerr << "[UnitreePointCloudReader] cloud stale - cleared\n";
            cloud_buf.Clear();
        }
    }
}

} // namespace kist
