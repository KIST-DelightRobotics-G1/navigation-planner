#include "lio/lio_receiver.hpp"

#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <cmath>
#include <cstring>
#include <iostream>

namespace kist {

namespace {

constexpr uint8_t kPointFieldFloat32 = 7;  // sensor_msgs/PointField datatype

// Offset of a named FLOAT32 field in a PointCloud2, or -1 if absent.
int float_field_offset(const std::vector<sensor_msgs::msg::dds_::PointField_>& fields,
                       const char* name) {
    for (const auto& f : fields)
        if (f.datatype() == kPointFieldFloat32 && f.name() == name)
            return int(f.offset());
    return -1;
}

int64_t stamp_ns(const builtin_interfaces::msg::dds_::Time_& t) {
    return int64_t(t.sec()) * 1000000000LL + t.nanosec();
}

} // namespace

LioReceiver::LioReceiver() = default;   // ChannelSubscriber is complete here
LioReceiver::~LioReceiver() { stop(); }

bool LioReceiver::start(int domain_id, const std::string& network_interface,
                        const std::string& odom_topic, const std::string& cloud_topic) {
    try {
        // No-op after the first Init in the same process.
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

        odom_sub_.reset(new OdomSub(odom_topic));
        odom_sub_->InitChannel([this](const void* m) { on_odometry(m); }, 1);

        cloud_sub_.reset(new CloudSub(cloud_topic));
        cloud_sub_->InitChannel([this](const void* m) { on_cloud(m); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[LioReceiver] DDS init failed on interface \"" << network_interface
                  << "\": " << e.what() << "\n  Is the LIO engine running on this DDS domain?\n";
        return false;
    }
    std::cout << "[LioReceiver] started on domain=" << domain_id
              << " odom=" << odom_topic << " cloud=" << cloud_topic << "\n";
    return true;
}

void LioReceiver::stop() {
    odom_sub_.reset();
    cloud_sub_.reset();
}

void LioReceiver::on_odometry(const void* message) {
    const auto& msg = *static_cast<const nav_msgs::msg::dds_::Odometry_*>(message);
    const auto& p = msg.pose().pose().position();
    const auto& q = msg.pose().pose().orientation();
    const auto& v = msg.twist().twist().linear();

    LioOdometry out;
    out.pose.stamp_ns = stamp_ns(msg.header().stamp());
    out.pose.parent   = FrameId::Odom;
    out.pose.child    = FrameId::Lidar;
    out.pose.T_parent_child.translation = Eigen::Vector3d(p.x(), p.y(), p.z());
    out.pose.T_parent_child.rotation =
        Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).normalized();
    out.linear_velocity = Eigen::Vector3d(v.x(), v.y(), v.z());

    odom_buf.SetData(std::move(out));
}

void LioReceiver::on_cloud(const void* message) {
    const auto& msg = *static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(message);
    const auto& fields = msg.fields();
    const int ox = float_field_offset(fields, "x");
    const int oy = float_field_offset(fields, "y");
    const int oz = float_field_offset(fields, "z");
    if (ox < 0 || oy < 0 || oz < 0) {
        std::cerr << "[LioReceiver] registered cloud without FLOAT32 x/y/z - dropped\n";
        return;
    }
    const int oi = float_field_offset(fields, "intensity");

    LioCloud out;
    out.stamp_ns = stamp_ns(msg.header().stamp());
    out.frame_id = msg.header().frame_id();

    const std::size_t n      = std::size_t(msg.width()) * msg.height();
    const std::size_t stride = msg.point_step();
    const uint8_t*    blob   = msg.data().data();
    out.xyz.reserve(n * 3);
    if (oi >= 0) out.intensity.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
        const uint8_t* rec = blob + i * stride;
        float x, y, z;
        std::memcpy(&x, rec + ox, 4);
        std::memcpy(&y, rec + oy, 4);
        std::memcpy(&z, rec + oz, 4);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        out.xyz.insert(out.xyz.end(), {x, y, z});
        if (oi >= 0) { float v; std::memcpy(&v, rec + oi, 4); out.intensity.push_back(v); }
    }

    cloud_buf.SetData(std::move(out));
}

} // namespace kist
