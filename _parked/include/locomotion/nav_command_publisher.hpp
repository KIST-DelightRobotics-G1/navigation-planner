#pragma once

// Worker: turn the planned path + the calibrated pose into a base-frame velocity
// command (vx, vy, vyaw) and publish it as a ROS Twist over DDS at rate_hz.
//
// Control law: pure pursuit while translating (steer toward a lookahead point on
// the path, slow down while turning and near the goal), then rotate in place to
// the goal yaw on arrival. Publishes zeros (stop) when there's no path, no pose,
// or a halt is set (the stop-and-ask behaviour). Downstream is
// kist-gearsonic-inference, which consumes the identical NavCommand contract.

#include "common/data_buffer.hpp"
#include "locomotion/nav_command.hpp"
#include "planner/planner.hpp"                 // PlanResult (carries the Path)
#include "kalman_filter/calibrated_pose.hpp"   // CalibratedPose (world x/y/yaw)

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace unitree::robot { template <typename T> class ChannelPublisher; }
namespace geometry_msgs::msg::dds_ { class Twist_; }

namespace kist {

inline constexpr const char* kNavCmdTopic = "rt/kist/nav/cmd_vel";

struct FollowConfig {
    double v_max         = 0.5;    // m/s cruise speed
    double vyaw_max      = 0.8;    // rad/s max turn rate
    double lookahead_m   = 0.6;    // pure-pursuit lookahead distance
    double arrival_tol_m = 0.15;   // within this of the goal -> stop translating, align yaw
    double yaw_tol_rad   = 0.087;  // ~5 deg; within this of the goal yaw -> arrived (stop)
    double k_yaw         = 1.5;    // steering / rotate proportional gain
    double slow_radius_m = 0.5;    // taper forward speed within this of the goal
    double rate_hz       = 20.0;   // publish cadence
    bool   allow_strafe  = true;   // holonomic: move toward the lookahead on both axes (vx+vy),
                                   // not car-like (vx only). vyaw still turns to face, so strafe fades.
};

class NavCommandPublisher {
public:
    NavCommandPublisher();    // ctor + dtor are out-of-line: the unique_ptr<ChannelPublisher>
    ~NavCommandPublisher();   // member needs the complete type, only visible in the .cpp

    NavCommandPublisher(const NavCommandPublisher&) = delete;
    NavCommandPublisher& operator=(const NavCommandPublisher&) = delete;

    // Open the DDS channel and start the publish loop off the plan + pose buffers.
    // False if the channel init fails / already running.
    bool start(int domain_id, const std::string& network_interface,
               DataBuffer<PlanResult>& plan_src, DataBuffer<CalibratedPose>& pose_src,
               const FollowConfig& cfg, const std::string& topic = kNavCmdTopic);
    void stop();

    // Finish heading (rad, world frame) to align to after reaching the goal.
    // Until set, arrival just stops (no rotate phase).
    void set_goal_yaw(float yaw_rad);
    void clear_goal_yaw();

    // Force a stop (publish zeros) regardless of the path — the stop-and-ask
    // behaviour holds this while waiting for permission.
    void set_halt(bool halt) { halt_.store(halt); }

    DataBuffer<NavCommand>& result() { return result_; }   // latest command (for viz / logging)
    bool running() const { return running_; }

    // Follow phase: 0 = driving, 1 = aligning yaw at the goal, 2 = arrived (done).
    // For the viewer overlay + the orchestrator (arrival triggers the next action).
    enum class Phase { Driving = 0, Aligning = 1, Arrived = 2 };
    Phase phase() const { return Phase(phase_.load()); }

private:
    void       run();
    void       publish(const NavCommand& c);
    NavCommand compute(const Path& path, const CalibratedPose& pose) const;

    using TwistPub = unitree::robot::ChannelPublisher<geometry_msgs::msg::dds_::Twist_>;
    std::unique_ptr<TwistPub> pub_;

    DataBuffer<PlanResult>*     plan_src_ = nullptr;
    DataBuffer<CalibratedPose>* pose_src_ = nullptr;
    FollowConfig                cfg_;

    mutable std::mutex yaw_mtx_;
    bool  has_goal_yaw_ = false;
    float goal_yaw_ = 0.0f;
    std::atomic<bool> halt_{false};

    mutable std::atomic<int> phase_{0};   // Phase; set in compute(), read by viewer/orchestrator
    DataBuffer<NavCommand> result_;
    std::thread           thread_;
    std::atomic<bool>     running_{false};
};

} // namespace kist
