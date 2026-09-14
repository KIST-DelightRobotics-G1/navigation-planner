#pragma once

// NavSystem — the deployment facade (mirrors kist-gearsonic-inference's GearsonicInference).
// Owns the readers/producer + the RoutePlanner computation + the DDS publisher, and runs the
// threads (folder separation is code organization; threading lives here):
//
//   perception thread : registered scan + stamp-matched pose -> RoutePlanner::update_map
//                       -> grid + costmap snapshots on buffers
//   planner thread    : costmap + goal -> RoutePlanner::plan -> path buffer
//   viz thread        : buffers -> rviz (grid / costmap / clearance / medial / path)
//
// PERCEPTION + PLANNING + VIZ only — it does NOT command the robot (no Twist). The local
// controller (path -> velocity) is added later, behind its own safety.

#include "common/data_buffer.hpp"
#include "controller/path_follower.hpp"
#include "controller/nav_command_publisher.hpp"
#include "goal_generation/destination_publisher.hpp"
#include "goal_generation/goal_command_receiver.hpp"
#include "lio/lio_receiver.hpp"
#include "lio/lio_transform_producer.hpp"
#include "mapping/obstacle_grid_publisher.hpp"
#include "planning/goal_receiver.hpp"
#include "route_planner/route_planner.hpp"

#include <atomic>
#include <optional>
#include <string>
#include <thread>

namespace kist {

class NavSystem {
public:
    NavSystem() = default;
    ~NavSystem() { stop(); }
    NavSystem(const NavSystem&) = delete;
    NavSystem& operator=(const NavSystem&) = delete;

    bool start(const std::string& config_path);
    void stop();

    void install_signal_handlers();
    void request_quit() { quit_ = true; }
    bool quit_requested() const { return quit_; }

private:
    void perception_run();
    void planner_run();
    void controller_run();
    void viz_run();

    // Freshest goal from either channel: rviz ad-hoc (gr_) or named command (goalcmd_).
    std::optional<Goal> active_goal();

    RoutePlanner          route_;
    PathFollower          follower_;
    LioTransformProducer  prod_;
    LioReceiver           rx_;
    GoalReceiver          gr_;         // rviz "2D Goal Pose" (rt/goal_pose) — ad-hoc, no dock
    DestinationPublisher  destpub_;    // advertises the catalog (rt/kist/nav/destinations)
    GoalCommandReceiver   goalcmd_;    // named goals from a peer/LLM (rt/kist/nav/goal)
    ObstacleGridPublisher pub_;
    NavCommandPublisher   cmd_pub_;
    bool                  drive_enabled_ = false;   // NAV_DRIVE=1 arms Twist output

    DataBuffer<ObstacleGrid> grid_buf_;
    DataBuffer<Costmap>      costmap_buf_;
    DataBuffer<Path>         path_buf_;
    DataBuffer<NavCommand>   cmd_buf_;      // follower output (NOT sent to the robot in this build)

    std::thread       perc_thread_, plan_thread_, ctrl_thread_, viz_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> quit_{false};

    bool sr_started_{false}, rx_started_{false}, gr_started_{false},
         destpub_started_{false}, goalcmd_started_{false}, pub_started_{false};
};

} // namespace kist
