#pragma once

// NavSystem — the deployment facade / ASSEMBLER (mirrors kist-gearsonic-inference's
// GearsonicInference). It owns the shared resources (readers, producer, publishers, buffers,
// goal channels) and the workers, wires them together, and start/stops them. There are NO loops
// or control logic here — each worker owns its thread + loop, and the compute logic lives in the
// domain stages (ObstacleMapper / RoutePlanner / LocalController) the workers drive:
//
//   perception : scan + pose      -> ObstacleMapper  -> grid + costmap buffers
//   planner    : costmap + goal   -> RoutePlanner    -> path buffer
//   controller : path+pose+cm+goal -> LocalController -> NavCommand (-> Twist if NAV_DRIVE=1)
//   viz        : buffers          -> rviz

#include "common/data_buffer.hpp"
#include "controller/nav_command.hpp"
#include "controller/nav_command_publisher.hpp"
#include "goal_generation/destination_publisher.hpp"
#include "goal_generation/goal_command_receiver.hpp"
#include "lio/lio_receiver.hpp"
#include "lio/lio_transform_producer.hpp"
#include "localization/localization_worker.hpp"
#include "localization/map_odom.hpp"
#include "localization/uwb_receiver.hpp"
#include "mapping/obstacle_grid_publisher.hpp"
#include "planning/goal_receiver.hpp"
#include "route_planner/perception/costmap_builder/costmap.hpp"
#include "route_planner/perception/obstacle_grid_builder/obstacle_grid.hpp"
#include "route_planner/perception/perception_worker.hpp"
#include "route_planner/planner/astar_planner/path.hpp"
#include "route_planner/planner/planner_worker.hpp"
#include "controller/controller_worker.hpp"
#include "system/goal_source.hpp"
#include "system/viz_worker.hpp"

#include <atomic>
#include <string>

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
    // ── shared resources (owned here, injected into the workers) ──
    LioTransformProducer  prod_;
    LioReceiver           rx_;
    GoalReceiver          gr_;         // rviz "2D Goal Pose" (rt/goal_pose) — ad-hoc, no dock
    DestinationPublisher  destpub_;    // advertises the catalog (rt/kist/nav/destinations)
    GoalCommandReceiver   goalcmd_;    // named goals from a peer/LLM (rt/kist/nav/goal)
    UwbReceiver           uwb_;        // UWB fixes (rt/kist/uwb/pose) — localization seed only
    ObstacleGridPublisher pub_;
    NavCommandPublisher   cmd_pub_;

    DataBuffer<ObstacleGrid> grid_buf_;
    DataBuffer<Costmap>      costmap_buf_;
    DataBuffer<Path>         path_buf_;
    DataBuffer<NavCommand>   cmd_buf_;
    DataBuffer<MapOdom>      mapodom_buf_;   // relocalizer output (map->odom); empty until locked
    GoalSource               goal_src_;

    // ── workers (each owns its thread + loop; driven off the buffers above) ──
    LocalizationWorker loc_;
    PerceptionWorker   perc_;
    PlannerWorker      plan_;
    ControllerWorker   ctrl_;
    VizWorker          viz_;

    std::atomic<bool> quit_{false};

    bool sr_started_{false}, rx_started_{false}, gr_started_{false},
         destpub_started_{false}, goalcmd_started_{false}, uwb_started_{false},
         pub_started_{false}, cmd_pub_started_{false};
};

} // namespace kist
