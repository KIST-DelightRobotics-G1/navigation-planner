#include "locomotion/nav_command_publisher.hpp"

#include <unitree/idl/ros2/Twist_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace kist {

namespace {
double wrap_pi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}  // namespace

NavCommandPublisher::NavCommandPublisher() = default;
NavCommandPublisher::~NavCommandPublisher() { stop(); }

bool NavCommandPublisher::start(int domain_id, const std::string& network_interface,
                                DataBuffer<PlanResult>& plan_src,
                                DataBuffer<CalibratedPose>& pose_src,
                                const FollowConfig& cfg, const std::string& topic) {
    if (running_) return true;
    try {
        // No-op if the embedding process already initialized the factory.
        unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
        pub_.reset(new TwistPub(topic));
        pub_->InitChannel();
    } catch (const std::exception& e) {
        std::cerr << "[NavCommandPublisher] DDS init failed on interface \""
                  << network_interface << "\": " << e.what() << "\n";
        return false;
    }
    plan_src_ = &plan_src;
    pose_src_ = &pose_src;
    cfg_      = cfg;
    running_  = true;
    thread_   = std::thread(&NavCommandPublisher::run, this);
    return true;
}

void NavCommandPublisher::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
    if (pub_) publish(NavCommand{});   // one last stop on the wire
    pub_.reset();
}

void NavCommandPublisher::set_goal_yaw(float yaw_rad) {
    std::lock_guard<std::mutex> lk(yaw_mtx_);
    has_goal_yaw_ = true;
    goal_yaw_ = yaw_rad;
}

void NavCommandPublisher::clear_goal_yaw() {
    std::lock_guard<std::mutex> lk(yaw_mtx_);
    has_goal_yaw_ = false;
}

// Pure pursuit -> base-frame (vx, vy, vyaw); rotate in place to the goal yaw once
// arrived. Returns zeros (stop) when there's no path / already arrived.
NavCommand NavCommandPublisher::compute(const Path& path, const CalibratedPose& pose) const {
    NavCommand cmd;   // zeros = stop
    if (path.waypoints.empty()) { phase_.store(int(Phase::Driving)); return cmd; }

    const double rx = pose.x, ry = pose.y, ryaw = pose.yaw;
    const auto&  goal = path.waypoints.back();
    const double dist_goal = std::hypot(goal.first - rx, goal.second - ry);

    // ── arrived: rotate in place to the finish heading ──
    if (dist_goal <= cfg_.arrival_tol_m) {
        float gy; bool has;
        { std::lock_guard<std::mutex> lk(yaw_mtx_); has = has_goal_yaw_; gy = goal_yaw_; }
        if (!has) { phase_.store(int(Phase::Arrived)); return cmd; }   // no finish heading -> arrived
        const double yaw_err = wrap_pi(double(gy) - ryaw);
        if (std::abs(yaw_err) <= cfg_.yaw_tol_rad) { phase_.store(int(Phase::Arrived)); return cmd; }
        phase_.store(int(Phase::Aligning));                            // rotating to the finish yaw
        cmd.vyaw = std::clamp(cfg_.k_yaw * yaw_err, -cfg_.vyaw_max, cfg_.vyaw_max);
        return cmd;
    }

    // ── translate: steer toward a lookahead point on the path ──
    // nearest waypoint to the robot
    size_t ni = 0;
    double best = 1e18;
    for (size_t i = 0; i < path.waypoints.size(); ++i) {
        const double dx = path.waypoints[i].first - rx, dy = path.waypoints[i].second - ry;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best) { best = d2; ni = i; }
    }
    // walk ahead until the accumulated distance reaches the lookahead (or the end)
    size_t li  = ni;
    double acc = 0.0;
    for (size_t i = ni + 1; i < path.waypoints.size(); ++i) {
        acc += std::hypot(path.waypoints[i].first  - path.waypoints[i - 1].first,
                          path.waypoints[i].second - path.waypoints[i - 1].second);
        li = i;
        if (acc >= cfg_.lookahead_m) break;
    }
    const auto&  look = path.waypoints[li];
    const double dx = look.first - rx, dy = look.second - ry;

    // world -> robot base frame
    const double c = std::cos(ryaw), s = std::sin(ryaw);
    const double fwd  =  c * dx + s * dy;
    const double left = -s * dx + c * dy;
    const double head_err = std::atan2(left, fwd);

    // Turn to face the lookahead; the strafe below naturally fades as it aligns.
    cmd.vyaw = std::clamp(cfg_.k_yaw * head_err, -cfg_.vyaw_max, cfg_.vyaw_max);
    double speed = cfg_.v_max *
                   std::min(1.0, dist_goal / std::max(1e-3, cfg_.slow_radius_m));   // taper near the goal
    const double mag = std::hypot(fwd, left);
    if (mag > 1e-6) {
        if (cfg_.allow_strafe) {                       // holonomic: move on both axes toward the lookahead
            cmd.vx = speed * fwd  / mag;
            cmd.vy = speed * left / mag;
        } else {                                       // car-like: forward only, slowed while turning
            cmd.vx = speed * std::max(0.0, fwd / mag);
            cmd.vy = 0.0;
        }
    }
    phase_.store(int(Phase::Driving));
    return cmd;
}

void NavCommandPublisher::publish(const NavCommand& c) {
    if (!pub_) return;
    geometry_msgs::msg::dds_::Twist_ msg;
    msg.linear().x(c.vx);  msg.linear().y(c.vy);  msg.linear().z(0.0);
    msg.angular().x(0.0);  msg.angular().y(0.0);  msg.angular().z(c.vyaw);
    pub_->Write(msg);
}

void NavCommandPublisher::run() {
    pthread_setname_np(pthread_self(), "nav-cmd-pub");
    using clock = std::chrono::steady_clock;
    const auto period =
        std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(1.0 / std::max(1.0, cfg_.rate_hz)));

    while (running_) {
        const auto t0 = clock::now();

        NavCommand cmd;   // zeros = stop (default)
        bool computed = false;
        if (!halt_.load()) {
            auto plan = plan_src_->GetData();
            auto pose = pose_src_->GetData();
            if (plan && pose && plan->has_goal) { cmd = compute(plan->path, *pose); computed = true; }
        }
        if (!computed) phase_.store(int(Phase::Driving));   // no goal/pose/halt -> not arrived
        publish(cmd);
        result_.SetData(cmd);

        std::this_thread::sleep_until(t0 + period);
    }
}

} // namespace kist
