#include "mapping/obstacle_grid_publisher.hpp"

#include <unitree/idl/ros2/OccupancyGrid_.hpp>
#include <unitree/idl/ros2/PoseStamped_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace kist {

namespace {
void set_stamp(builtin_interfaces::msg::dds_::Time_& t, int64_t ns) {
    t.sec()     = int32_t(ns / 1000000000LL);
    t.nanosec() = uint32_t(ns % 1000000000LL);
}
}  // namespace

ObstacleGridPublisher::ObstacleGridPublisher() = default;   // ChannelPublisher complete here
ObstacleGridPublisher::~ObstacleGridPublisher() = default;

bool ObstacleGridPublisher::start(int domain_id, const std::string& network_interface,
                                  const std::string& grid_topic, const std::string& pose_topic,
                                  const std::string& frame_id) {
    frame_ = frame_id;
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);  // no-op if inited
        grid_pub_.reset(new GridPub(grid_topic));
        grid_pub_->InitChannel();
        pose_pub_.reset(new PosePub(pose_topic));
        pose_pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[ObstacleGridPublisher] DDS init failed: " << e.what() << "\n";
        return false;
    }
    std::cout << "[ObstacleGridPublisher] grid=" << grid_topic << " pose=" << pose_topic
              << " frame=" << frame_ << "\n";
    return true;
}

void ObstacleGridPublisher::publish(const ObstacleGrid& g, const ObstacleGridConfig& cfg) {
    if (g.empty() || !grid_pub_ || !pose_pub_) return;

    // ── nav_msgs/OccupancyGrid ────────────────────────────────────────────────
    // Row-major from the origin corner, x-major (index = iy*width + ix) — the same
    // layout as ObstacleGrid.index(). Values: -1 unknown, 0 free .. 100 occupied.
    // stamp = 0: rviz's tf2 message filter then uses the LATEST transform for `frame_`
    // instead of waiting for one at an exact time — this avoids the intermittent
    // "Message Filter dropping ... queue is full" drops (viz-only; the ObstacleGrid
    // struct keeps its real stamp_ns for actual consumers).
    nav_msgs::msg::dds_::OccupancyGrid_ og;
    og.header().frame_id() = frame_;
    set_stamp(og.header().stamp(), 0);
    og.info().resolution() = g.resolution;
    og.info().width()      = uint32_t(g.n);
    og.info().height()     = uint32_t(g.n);
    og.info().origin().position().x(g.origin_x);
    og.info().origin().position().y(g.origin_y);
    og.info().origin().position().z(0.0);
    og.info().origin().orientation().w(1.0);   // identity (axis-aligned in odom)

    auto& data = og.data();
    data.resize(std::size_t(g.n) * g.n);
    for (std::size_t i = 0; i < g.log_odds.size(); ++i) {
        const float l = g.log_odds[i];
        int8_t v;
        if (std::fabs(l) < 0.01f)      v = -1;                                  // unobserved
        else {
            const float p = 1.0f / (1.0f + std::exp(-l));
            v = int8_t(std::clamp(int(std::lround(p * 100.0f)), 0, 100));
        }
        data[i] = uint8_t(v);   // wire type is int8; -1 travels as 0xFF
    }
    grid_pub_->Write(og);

    // ── geometry_msgs/PoseStamped (robot base heading) ────────────────────────
    geometry_msgs::msg::dds_::PoseStamped_ ps;
    ps.header().frame_id() = frame_;
    set_stamp(ps.header().stamp(), 0);   // latest-transform (see OccupancyGrid stamp above)
    ps.pose().position().x(g.robot_x);
    ps.pose().position().y(g.robot_y);
    ps.pose().position().z(0.0);
    ps.pose().orientation().w(std::cos(g.robot_yaw * 0.5f));   // yaw about +Z
    ps.pose().orientation().x(0.0);
    ps.pose().orientation().y(0.0);
    ps.pose().orientation().z(std::sin(g.robot_yaw * 0.5f));
    pose_pub_->Write(ps);
}

} // namespace kist
