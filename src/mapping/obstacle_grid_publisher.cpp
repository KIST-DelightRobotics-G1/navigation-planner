#include "mapping/obstacle_grid_publisher.hpp"

#include <unitree/idl/ros2/OccupancyGrid_.hpp>
#include <unitree/idl/ros2/PoseStamped_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

namespace kist {

namespace {
void set_stamp(builtin_interfaces::msg::dds_::Time_& t, int64_t ns) {
    t.sec()     = int32_t(ns / 1000000000LL);
    t.nanosec() = uint32_t(ns % 1000000000LL);
}

// Fill a PointCloud2 (x,y,z float32) from odom points, z just above the map.
void make_cloud(sensor_msgs::msg::dds_::PointCloud2_& pc,
                const std::vector<std::pair<float, float>>& pts, const std::string& frame) {
    pc.header().frame_id() = frame;
    set_stamp(pc.header().stamp(), 0);
    pc.height(1);
    pc.width(uint32_t(pts.size()));
    pc.is_bigendian(false);
    pc.is_dense(true);
    pc.point_step(12);
    pc.row_step(uint32_t(12 * pts.size()));
    const char* names[3] = {"x", "y", "z"};
    for (int k = 0; k < 3; ++k) {
        sensor_msgs::msg::dds_::PointField_ f;
        f.name(names[k]); f.offset(uint32_t(4 * k)); f.datatype(7); f.count(1);
        pc.fields().push_back(f);
    }
    auto& blob = pc.data();
    blob.resize(pts.size() * 12);
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const float xyz[3] = {pts[i].first, pts[i].second, 0.02f};
        std::memcpy(&blob[i * 12], xyz, 12);
    }
}
}  // namespace

ObstacleGridPublisher::ObstacleGridPublisher() = default;   // ChannelPublisher complete here
ObstacleGridPublisher::~ObstacleGridPublisher() = default;

bool ObstacleGridPublisher::start(int domain_id, const std::string& network_interface,
                                  const std::string& grid_topic, const std::string& pose_topic,
                                  const std::string& costmap_topic, const std::string& path_topic,
                                  const std::string& raw_path_topic, const std::string& clearance_topic,
                                  const std::string& medial_topic, const std::string& frame_id) {
    frame_ = frame_id;
    try {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);  // no-op if inited
        grid_pub_.reset(new GridPub(grid_topic));
        grid_pub_->InitChannel();
        pose_pub_.reset(new PosePub(pose_topic));
        pose_pub_->InitChannel();
        costmap_pub_.reset(new GridPub(costmap_topic));
        costmap_pub_->InitChannel();
        path_pub_.reset(new PathPub(path_topic));
        path_pub_->InitChannel();
        raw_path_pub_.reset(new PathPub(raw_path_topic));
        raw_path_pub_->InitChannel();
        clearance_pub_.reset(new GridPub(clearance_topic));
        clearance_pub_->InitChannel();
        medial_pub_.reset(new PathPub(medial_topic));
        medial_pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[ObstacleGridPublisher] DDS init failed: " << e.what() << "\n";
        return false;
    }
    std::cout << "[ObstacleGridPublisher] grid=" << grid_topic << " pose=" << pose_topic
              << " costmap=" << costmap_topic << " path=" << path_topic << " raw=" << raw_path_topic
              << " clearance=" << clearance_topic << " medial=" << medial_topic
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

void ObstacleGridPublisher::publish_costmap(const Costmap& cm) {
    if (cm.empty() || !costmap_pub_) return;

    nav_msgs::msg::dds_::OccupancyGrid_ og;
    og.header().frame_id() = frame_;
    set_stamp(og.header().stamp(), 0);   // latest-transform (see publish() above)
    og.info().resolution() = cm.resolution;
    og.info().width()      = uint32_t(cm.n);
    og.info().height()     = uint32_t(cm.n);
    og.info().origin().position().x(cm.origin_x);
    og.info().origin().position().y(cm.origin_y);
    og.info().origin().position().z(0.0);
    og.info().origin().orientation().w(1.0);

    // cost 0..254 -> OccupancyGrid 0..100 (rviz costmap colour scheme reads this).
    auto& data = og.data();
    data.resize(std::size_t(cm.n) * cm.n);
    for (std::size_t i = 0; i < cm.cells.size(); ++i)
        data[i] = uint8_t(std::lround(cm.cells[i] * (100.0 / 254.0)));
    costmap_pub_->Write(og);
}

void ObstacleGridPublisher::publish_path(const std::vector<std::pair<float, float>>& wps,
                                         float spacing) {
    if (!path_pub_) return;

    // Densify: sample each segment at ~spacing so the point set reads as a line in rviz.
    std::vector<std::pair<float, float>> pts;
    if (wps.size() >= 2 && spacing > 0.0f) {
        for (std::size_t i = 0; i + 1 < wps.size(); ++i) {
            const float ax = wps[i].first,  ay = wps[i].second;
            const float bx = wps[i+1].first, by = wps[i+1].second;
            const float len = std::hypot(bx - ax, by - ay);
            const int steps = std::max(1, int(len / spacing));
            for (int s = 0; s < steps; ++s) {
                const float t = float(s) / steps;
                pts.emplace_back(ax + t*(bx-ax), ay + t*(by-ay));
            }
        }
        pts.push_back(wps.back());
    } else {
        pts = wps;
    }

    sensor_msgs::msg::dds_::PointCloud2_ pc;
    make_cloud(pc, pts, frame_);
    path_pub_->Write(pc);
}

void ObstacleGridPublisher::publish_path_raw(const std::vector<std::pair<float, float>>& wps) {
    if (!raw_path_pub_) return;
    sensor_msgs::msg::dds_::PointCloud2_ pc;
    make_cloud(pc, wps, frame_);   // A* grid path is already dense — publish as-is
    raw_path_pub_->Write(pc);
}

void ObstacleGridPublisher::publish_medial(const std::vector<std::pair<float, float>>& points) {
    if (!medial_pub_) return;
    sensor_msgs::msg::dds_::PointCloud2_ pc;
    make_cloud(pc, points, frame_);   // raw ridge points (no densify)
    medial_pub_->Write(pc);
}

void ObstacleGridPublisher::publish_clearance(const Costmap& cm, const std::vector<float>& clr,
                                              float max_m) {
    if (cm.empty() || int(clr.size()) != cm.n * cm.n || !clearance_pub_) return;

    nav_msgs::msg::dds_::OccupancyGrid_ og;
    og.header().frame_id() = frame_;
    set_stamp(og.header().stamp(), 0);
    og.info().resolution() = cm.resolution;
    og.info().width()      = uint32_t(cm.n);
    og.info().height()     = uint32_t(cm.n);
    og.info().origin().position().x(cm.origin_x);
    og.info().origin().position().y(cm.origin_y);
    og.info().origin().position().z(0.0);
    og.info().origin().orientation().w(1.0);

    auto& data = og.data();
    data.resize(clr.size());
    const float inv = (max_m > 0.0f) ? (100.0f / max_m) : 0.0f;   // clearance (m) -> 0..100 heatmap
    for (std::size_t i = 0; i < clr.size(); ++i)
        data[i] = uint8_t(std::clamp(int(clr[i] * inv), 0, 100));
    clearance_pub_->Write(og);
}

} // namespace kist
