#include "unitree/unitree_odometry_reader.hpp"

#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <iostream>

namespace kist {

// ─── Odometry (ROS2 IDL) → UnitreeOdometry ────────────────────────────────────
// Fixed-size member copy (gearsonic convert() style) — nothing to
// validate, Odometry has neither a CRC nor optional fields.

namespace {

UnitreeOdometry convert(const nav_msgs::msg::dds_::Odometry_& msg) {
    UnitreeOdometry out;
    out.stamp_ns = int64_t(msg.header().stamp().sec()) * 1000000000LL +
                   msg.header().stamp().nanosec();
    out.frame_id = msg.header().frame_id();

    const auto& p = msg.pose().pose();
    out.px = float(p.position().x());
    out.py = float(p.position().y());
    out.pz = float(p.position().z());
    out.qx = float(p.orientation().x());
    out.qy = float(p.orientation().y());
    out.qz = float(p.orientation().z());
    out.qw = float(p.orientation().w());

    const auto& t = msg.twist().twist();
    out.vx = float(t.linear().x());
    out.vy = float(t.linear().y());
    out.vz = float(t.linear().z());
    out.wx = float(t.angular().x());
    out.wy = float(t.angular().y());
    out.wz = float(t.angular().z());
    return out;
}

} // namespace

// ─── UnitreeOdometryReader ────────────────────────────────────────────────────

UnitreeOdometryReader& UnitreeOdometryReader::instance() {
    static UnitreeOdometryReader inst;
    return inst;
}

bool UnitreeOdometryReader::start(int domain_id, const std::string& network_interface,
                                  const std::string& topic) {
    try {
        // Safe when the embedding process already initialized the factory
        // (Init is a no-op after the first call in the same process).
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

        odom_sub_.reset(new OdomSub(topic));
        odom_sub_->InitChannel(
            [this](const void* msg) { on_odom_update(msg); }, 1);
    } catch (const std::exception& e) {
        std::cerr << "[UnitreeOdometryReader] DDS init failed on interface \""
                  << network_interface << "\": " << e.what()
                  << "\n  Check the robot LAN cable.\n";
        return false;
    }

    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&UnitreeOdometryReader::watchdog_loop, this);
    std::cout << "[UnitreeOdometryReader] started on domain=" << domain_id
              << " interface=" << network_interface
              << " topic=" << topic << "\n";
    return true;
}

void UnitreeOdometryReader::stop() {
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    odom_sub_.reset();
}

void UnitreeOdometryReader::on_odom_update(const void* message) {
    odom_buf.SetData(convert(*static_cast<const nav_msgs::msg::dds_::Odometry_*>(message)));
}

// Odometry streams at ~50Hz (measured) over BestEffort, where sporadic
// drops are normal — 200ms (10 frames) of silence marks a genuinely
// dead stream without tripping on single losses. Polled at 10ms, same
// cadence as the other readers' watchdogs.
void UnitreeOdometryReader::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 200.0;  // 10 frames at 50Hz

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(10ms);

        auto odom = odom_buf.GetDataWithTime();
        if (odom.HasData() && odom.GetAgeMs() > stale_ms) {
            std::cerr << "[UnitreeOdometryReader] odometry stale - cleared\n";
            odom_buf.Clear();
        }
    }
}

} // namespace kist
