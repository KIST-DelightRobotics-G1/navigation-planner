cmake_minimum_required(VERSION 3.18)
project(kist_navigation_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)       # header-only; linear algebra
find_package(PCL REQUIRED COMPONENTS common io filters registration)  # localization: prior-map GICP
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# CUDAToolkit / OpenCV dropped — only the parked CV/occupancy stack needed them.

# ─── thirdparty: unitree_sdk2 ─────────────────────────────────────────────────
# Guarded so an embedding super-project that already provides the target
# (e.g. via kist-gearsonic-inference) composes without a collision.
if(NOT TARGET unitree_sdk2)
    set(BUILD_EXAMPLES OFF CACHE BOOL "Build unitree_sdk2 examples" FORCE)
    add_subdirectory(thirdparty/unitree_sdk2)
endif()

# ─── kist_nav DDS contract ────────────────────────────────────────────────────
# Nav wire types (rt/kist/nav/*), generated at build time from idl/kist_nav.idl
# via the CycloneDDS idlc toolchain (0.10.2, matching the SDK's bundled ddscxx),
# same pattern as kist-gearsonic-inference / kist-ext-sensor-io. Gated on idlc:
# absent -> the goal_generation DDS layer is skipped and the repo still builds.
find_program(CYCLONE_IDLC idlc HINTS /opt/cyclonedds/bin $ENV{CYCLONEDDS_HOME}/bin)
if(CYCLONE_IDLC)
    set(KIST_NAV_IDL_GEN ${CMAKE_BINARY_DIR}/idl_gen)
    file(MAKE_DIRECTORY ${KIST_NAV_IDL_GEN})
    add_custom_command(
        OUTPUT ${KIST_NAV_IDL_GEN}/kist_nav.hpp ${KIST_NAV_IDL_GEN}/kist_nav.cpp
        COMMAND ${CYCLONE_IDLC} -l cxx ${CMAKE_CURRENT_SOURCE_DIR}/idl/kist_nav.idl
        WORKING_DIRECTORY ${KIST_NAV_IDL_GEN}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/idl/kist_nav.idl
        COMMENT "idlc: kist_nav.idl -> C++ DDS types")
    add_library(kist_nav_idl STATIC ${KIST_NAV_IDL_GEN}/kist_nav.cpp)
    target_include_directories(kist_nav_idl PUBLIC ${KIST_NAV_IDL_GEN})
    target_link_libraries(kist_nav_idl PUBLIC unitree_sdk2)   # bundled ddscxx

    # ─── goal_generation ──────────────────────────────────────────────────────
    # Mission/goal layer. First piece: publish the destination catalog over DDS.
    add_library(goal_generation STATIC
        src/goal_generation/destination_publisher.cpp
        src/goal_generation/goal_command_receiver.cpp)
    target_include_directories(goal_generation PUBLIC include)
    target_link_libraries(goal_generation PUBLIC kist_nav_idl yaml-cpp)

    add_executable(test_destination_publisher test/test_destination_publisher.cpp)
    target_link_libraries(test_destination_publisher PRIVATE goal_generation yaml-cpp)

    add_executable(test_destination_sub test/test_destination_sub.cpp)
    target_link_libraries(test_destination_sub PRIVATE goal_generation yaml-cpp)

    add_executable(test_subtask_cmd_send test/test_subtask_cmd_send.cpp)
    target_link_libraries(test_subtask_cmd_send PRIVATE goal_generation yaml-cpp)

    add_executable(test_goal_recv test/test_goal_recv.cpp)
    target_link_libraries(test_goal_recv PRIVATE goal_generation yaml-cpp)

    # SubtaskState subscriber — watch the nav state back-channel (rt/cortex/nav/state).
    add_executable(test_subtask_state_sub test/test_subtask_state_sub.cpp)
    target_link_libraries(test_subtask_state_sub PRIVATE goal_generation yaml-cpp)
else()
    message(STATUS "CycloneDDS idlc not found — skipping kist_nav DDS types (goal_generation)")
endif()

# unitree_pointcloud_reader + unitree_pointcloud_imu_reader (the utlidar cloud/IMU
# feeders for the old in-planner LIO) are RETIRED under _parked/ — LIO now runs as an
# external engine and the planner consumes its output via lio_receiver.

# ─── unitree_odometry_reader ──────────────────────────────────────────────────
# Raw locomotion-controller odometry (rt/dog_odom); odom-frame, drifts —
# the EKF fusion stage (odom predict + UWB update) owns global pose.
add_library(unitree_odometry_reader STATIC
    src/unitree/unitree_odometry_reader.cpp
)
target_include_directories(unitree_odometry_reader PUBLIC include)
target_link_libraries(unitree_odometry_reader PUBLIC unitree_sdk2)

# ─── unitree_state_reader ───────────────────────────────────────────────────────
# G1 lowstate (rt/lowstate) -> waist joints, for FK sensor-bob compensation.
add_library(unitree_state_reader STATIC
    src/unitree/unitree_state_reader.cpp
)
target_include_directories(unitree_state_reader PUBLIC include)
target_link_libraries(unitree_state_reader PUBLIC unitree_sdk2)

add_executable(test_lowstate_probe test/test_lowstate_probe.cpp)
target_link_libraries(test_lowstate_probe PRIVATE unitree_state_reader yaml-cpp)

# ─── transforms / frames ──────────────────────────────────────────────────────
# SE(3) transform math + the robot frame tree (waist FK, sensor extrinsics). The
# foundation for state estimation / deskew and the LIO transform tree.
add_library(transforms STATIC
    src/transforms/rigid.cpp
    src/transforms/pose_interpolator.cpp
    src/transforms/pose_history.cpp
    src/transforms/transform_history.cpp
    src/transforms/transform_tree.cpp)
target_include_directories(transforms PUBLIC include)
target_link_libraries(transforms PUBLIC Eigen3::Eigen)

# frames/ vocabulary is header-only (frame_ids.hpp, frame_convention.hpp); this lib
# adds only the static-extrinsics loader (the measured, fixed edges of the tree).
add_library(frames STATIC src/frames/sensor_extrinsics.cpp)
target_include_directories(frames PUBLIC include)
target_link_libraries(frames PUBLIC transforms yaml-cpp)

# lio/ — the planner's single receiver for the external LIO engine (docs/LIO_ENGINE.md):
# subscribes /Odometry_loc + /cloud_registered_1 -> LioOdometry / LioCloud buffers.
add_library(lio_receiver STATIC src/lio/lio_receiver.cpp)
target_include_directories(lio_receiver PUBLIC include)
target_link_libraries(lio_receiver PUBLIC transforms unitree_sdk2)

# kinematics/ — G1 forward kinematics (waist FK: pelvis->torso from joints 12/13/14).
add_library(kinematics STATIC src/kinematics/g1_kinematics.cpp)
target_include_directories(kinematics PUBLIC include)
target_link_libraries(kinematics PUBLIC transforms)

# lio_transform_producer — compose LIO pose + waist FK -> T_odom_lidar / T_odom_pelvis
# (RobotTransforms) into a thread-safe buffer + a stamp ring; consumers read either.
add_library(lio_transform_producer STATIC src/lio/lio_transform_producer.cpp)
target_include_directories(lio_transform_producer PUBLIC include)
target_link_libraries(lio_transform_producer PUBLIC transforms kinematics)

# route_planner/ — the route-planning pipeline, one folder per stage + the orchestrator:
#   obstacle_grid_builder : registered LIO scan (odom) -> 3D log-odds voxel grid + rolling
#   costmap_builder       : occupancy -> EDT inflation costmap (+ clearance / medial axis)
#   astar_planner         : clearance-centred A* route (raw grid path)
#   route_smoothing       : LOS straighten + max-radius circular-arc corners (the shape)
#   route_planner.{hpp,cpp}: wires the four stages (update_map + plan). Pure computation, no
#   threads / DDS (folder split is organization only). LiDAR-only for now (camera last).
add_library(route_planner STATIC
    src/route_planner/perception/obstacle_grid_builder/obstacle_voxel_grid.cpp
    src/route_planner/perception/costmap_builder/costmap_builder.cpp
    src/route_planner/perception/costmap_builder/edt.cpp
    src/route_planner/perception/costmap_builder/clearance.cpp
    src/route_planner/perception/obstacle_mapper.cpp
    src/route_planner/planner/astar_planner/astar_planner.cpp
    src/route_planner/planner/route_smoothing/route_smoother.cpp
    src/route_planner/planner/route_planner.cpp)
target_include_directories(route_planner PUBLIC include)
target_link_libraries(route_planner PUBLIC Eigen3::Eigen)   # Transform (LioCloud is header-only)

# obstacle_grid_publisher — bridge the grid/costmap/path to rviz2 over DDS (OccupancyGrid /
# PoseStamped / PointCloud2). Visualization only; the planner reads the structs directly.
add_library(obstacle_grid_publisher STATIC src/mapping/obstacle_grid_publisher.cpp)
target_include_directories(obstacle_grid_publisher PUBLIC include)
target_link_libraries(obstacle_grid_publisher PUBLIC route_planner unitree_sdk2)

# goal_receiver — rviz "2D Goal Pose" subscriber (rt/goal_pose), ad-hoc goals (dock off).
# Subtask goals (rt/cortex/nav/cmd, with dock) come from goal_generation. Its header pulls in the
# unified kist::Goal from goal_generation/goal.hpp (header-only), so no extra link is needed.
add_library(goal_receiver STATIC src/planning/goal_receiver.cpp)
target_include_directories(goal_receiver PUBLIC include)
target_link_libraries(goal_receiver PUBLIC unitree_sdk2)

# controller/ — pure-pursuit path follower (Path + pose -> NavCommand) + reactive-stop
# safety, and the Twist publisher (NavCommand -> rt/kist/nav/cmd_vel -> gearsonic).
add_library(controller STATIC
    src/controller/path_follower.cpp
    src/controller/local_controller.cpp
    src/controller/nav_command_publisher.cpp)
target_include_directories(controller PUBLIC include)
target_link_libraries(controller PUBLIC route_planner unitree_sdk2)   # Costmap + DDS Twist
# SubtaskStatePublisher (rt/cortex/nav/state) needs the generated kist_nav types, so it is only
# compiled into controller when idlc is available — same gate as goal_generation. Consumers of
# the status back-channel (nav_system) already require idlc, so this adds no new constraint.
if(CYCLONE_IDLC)
    target_sources(controller PRIVATE src/controller/subtask_state_publisher.cpp)
    target_link_libraries(controller PUBLIC kist_nav_idl)
endif()

# system/ — the deployment facade: owns the readers/producer + RoutePlanner + publisher, and
# runs the perception / planner / viz threads. The kist-navigation-planner binary runs this.
add_library(nav_system STATIC
    src/system/nav_system.cpp
    src/route_planner/perception/perception_worker.cpp   # workers live in their domain folders,
    src/route_planner/planner/planner_worker.cpp          # but the worker layer (lio/DDS/thread) is
    src/controller/controller_worker.cpp                  # compiled into nav_system so the domain
    src/localization/localization_worker.cpp              # libs stay pure computation (Eigen/PCL only).
    src/system/viz_worker.cpp)
target_include_directories(nav_system PUBLIC include)
target_link_libraries(nav_system PUBLIC
    route_planner controller goal_receiver goal_generation localization uwb_receiver
    obstacle_grid_publisher lio_receiver lio_transform_producer unitree_state_reader yaml-cpp)

add_executable(kist-navigation-planner src/main.cpp)
target_link_libraries(kist-navigation-planner PRIVATE nav_system)

# kinematics/ (waist+foot FK) + state_estimation/ (InEKF) are PARKED under _parked/:
# the LIO engine provides T_odom_lidar over DDS, so waist FK / InEKF aren't needed.
# (invariant-ekf + fast-lio submodules removed; the LIO engine is external — see below.)

# Transform SE(3) math + pose interpolation — pin the conventions with tests.
add_executable(test_transform test/test_transform.cpp)
target_include_directories(test_transform PRIVATE include)
target_link_libraries(test_transform PRIVATE Eigen3::Eigen)

add_executable(test_pose_history test/test_pose_history.cpp)
target_link_libraries(test_pose_history PRIVATE transforms)

# Frame tree — static + dynamic edge composition, time interpolation, deskew lookup.
add_executable(test_transform_tree test/test_transform_tree.cpp)
target_link_libraries(test_transform_tree PRIVATE transforms)

# Static-extrinsics loader — YAML load, name->FrameId, RPY->quat, rejections.
add_executable(test_sensor_extrinsics test/test_sensor_extrinsics.cpp)
target_link_libraries(test_sensor_extrinsics PRIVATE frames)

# LIO receiver — live /Odometry_loc + /cloud_registered_1 -> pose (pos+yaw+vel) + cloud.
add_executable(test_lio_receiver test/test_lio_receiver.cpp)
target_link_libraries(test_lio_receiver PRIVATE lio_receiver yaml-cpp)

# LIO transforms — LIO pose + waist FK -> T_odom_lidar (wobbles) / T_odom_pelvis (stable).
add_executable(test_lio_transforms test/test_lio_transforms.cpp)
target_link_libraries(test_lio_transforms PRIVATE
    lio_transform_producer lio_receiver unitree_state_reader yaml-cpp)

# Obstacle grid — live registered scan -> 2D log-odds occupancy (ray-carve + floor-band),
# stamp-matched pose. Publishes /obstacle_grid + /robot_pose for rviz2; floor probe to
# the terminal. Validate standing.
add_executable(test_obstacle_grid test/test_obstacle_grid.cpp)
target_link_libraries(test_obstacle_grid PRIVATE
    route_planner obstacle_grid_publisher goal_receiver lio_transform_producer lio_receiver
    unitree_state_reader yaml-cpp)

# recorder/ — the map-recording WORKER: accumulate registered scans (rt/cloud_registered_1) into a
# voxel grid + capture the map-origin UWB & odom pose P_B; save() writes the PCD + .uwb sidecar.
# Owns its thread + accumulation loop (same worker split as the runtime stages).
add_library(map_recorder STATIC src/recorder/map_recorder.cpp)
target_include_directories(map_recorder PUBLIC include)
target_link_libraries(map_recorder PUBLIC lio_receiver uwb_receiver)

# system/map_recorder — the ASSEMBLER (mirrors nav_system): owns the readers + the MapRecorder
# worker, wires + start/stops them. No loops here; the worker owns its thread.
add_library(map_recorder_system STATIC src/system/map_recorder.cpp)
target_include_directories(map_recorder_system PUBLIC include)
target_link_libraries(map_recorder_system PUBLIC map_recorder lio_receiver uwb_receiver yaml-cpp)

# kist-map-recorder — deployment entry (src/map_recorder_main.cpp, like src/main.cpp): run the
# lifecycle, Ctrl+C to save maps/map.pcd (+ maps/map.uwb) with an overwrite prompt.
add_executable(kist-map-recorder src/map_recorder_main.cpp)
target_link_libraries(kist-map-recorder PRIVATE map_recorder_system unitree_sdk2 yaml-cpp)

# localization/ — relocalizer: register a bounded local submap (from rt/cloud_registered_1,
# odom) against the prior PCD map (map) via PCL GICP -> map->odom. PCL is hidden behind a PIMPL.
add_library(localization STATIC
    src/localization/relocalizer.cpp
    src/localization/localization_filter.cpp)   # 3-state EKF: GICP + UWB -> T_map_odom (pure Eigen)
target_include_directories(localization PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(localization PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

# uwb_receiver — UWB fixes (rt/kist/uwb/pose, PoseStamped) for the localization seed. Own lib
# (DDS I/O) so the localization lib stays pure PCL computation. No custom IDL, no ext link.
add_library(uwb_receiver STATIC src/localization/uwb_receiver.cpp)
target_include_directories(uwb_receiver PUBLIC include)
target_link_libraries(uwb_receiver PUBLIC unitree_sdk2 Eigen3::Eigen)

# reloc_check — v0 validation: GICP a live submap onto the prior map from a manual seed (classic),
# or GLOBAL_INIT yaw-search from a live UWB fix + T_odom_pelvis (needs the producer/state/uwb libs);
# prints fitness/curve + T_map_odom and dumps reloc_overlay.pcd for a pcl_viewer overlap check.
add_executable(reloc_check test/tool_reloc_check.cpp)
target_link_libraries(reloc_check PRIVATE localization uwb_receiver lio_receiver
                      lio_transform_producer unitree_state_reader obstacle_grid_publisher
                      unitree_sdk2 yaml-cpp)

# ─── LIO engine: external DDS service (not built here) ────────────────────────
# LIO now runs as a separate ROS2 process (deepglint FAST_LIO_LOCALIZATION_HUMANOID,
# raw Livox driver), pinned by the docker image — see docs/LIO_ENGINE.md. It talks to
# this ROS-free planner ONLY over DDS: it publishes T_odom_lidar on /Odometry_loc
# (nav_msgs/Odometry, frame camera_init) and the registered cloud on /cloud_registered_1
# (sensor_msgs/PointCloud2, odom frame). The planner subscribes to those. Our earlier
# ROS-free FAST-LIO port (src/lio/, ikd-Tree, probes) is retired under _parked/.
