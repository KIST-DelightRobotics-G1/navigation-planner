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
// PointCloud2 has no CRC, so validation is structural: the FLOAT32 x/y/z
// fields must exist (looked up by name — their offsets are not assumed).
// Walks the binary blob by point_step and drops non-finite points.
// Returns false when x/y/z are missing; frame_out is then untouched.

namespace {

constexpr uint8_t kPointFieldFloat32 = 7;  // sensor_msgs/PointField datatype

// Byte offsets of the x/y/z fields inside one point record.
struct XYZLayout {
    uint32_t x = 0, y = 0, z = 0;
    bool found_x = false, found_y = false, found_z = false;
    bool complete() const { return found_x && found_y && found_z; }
};

XYZLayout find_xyz_layout(const std::vector<sensor_msgs::msg::dds_::PointField_>& fields) {
    XYZLayout layout;
    for (const auto& f : fields) {
        if (f.datatype() != kPointFieldFloat32)
            continue;
        if      (f.name() == "x") { layout.x = f.offset(); layout.found_x = true; }
        else if (f.name() == "y") { layout.y = f.offset(); layout.found_y = true; }
        else if (f.name() == "z") { layout.z = f.offset(); layout.found_z = true; }
    }
    return layout;
}

bool decode_pointcloud2(const sensor_msgs::msg::dds_::PointCloud2_& msg,
                        UnitreePointCloud& frame_out) {
    const XYZLayout layout = find_xyz_layout(msg.fields());
    if (!layout.complete())
        return false;

    frame_out.stamp_ns = int64_t(msg.header().stamp().sec()) * 1000000000LL +
                         msg.header().stamp().nanosec();
    frame_out.frame_id = msg.header().frame_id();

    const std::size_t n_points = std::size_t(msg.width()) * msg.height();
    const std::size_t stride   = msg.point_step();
    const uint8_t*    blob     = msg.data().data();

    frame_out.xyz.clear();
    frame_out.xyz.reserve(n_points * 3);

    for (std::size_t i = 0; i < n_points; ++i) {
        const uint8_t* rec = blob + i * stride;
        float x, y, z;
        std::memcpy(&x, rec + layout.x, sizeof(float));
        std::memcpy(&y, rec + layout.y, sizeof(float));
        std::memcpy(&z, rec + layout.z, sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;
        frame_out.xyz.insert(frame_out.xyz.end(), {x, y, z});
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
                                    const std::string& topic) {
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
    cloud_buf.SetData(std::move(frame));
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
