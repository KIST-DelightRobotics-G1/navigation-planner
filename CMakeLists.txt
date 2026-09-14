cmake_minimum_required(VERSION 3.18)
project(kist_navigation_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)       # header-only; linear algebra
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

    add_executable(test_goal_send test/test_goal_send.cpp)
    target_link_libraries(test_goal_send PRIVATE goal_generation yaml-cpp)

    add_executable(test_goal_recv test/test_goal_recv.cpp)
    target_link_libraries(test_goal_recv PRIVATE goal_generation yaml-cpp)
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
    src/route_planner/obstacle_grid_builder/obstacle_voxel_grid.cpp
    src/route_planner/costmap_builder/costmap_builder.cpp
    src/route_planner/costmap_builder/edt.cpp
    src/route_planner/costmap_builder/clearance.cpp
    src/route_planner/astar_planner/astar_planner.cpp
    src/route_planner/route_smoothing/route_smoother.cpp
    src/route_planner/route_planner.cpp)
target_include_directories(route_planner PUBLIC include)
target_link_libraries(route_planner PUBLIC Eigen3::Eigen)   # Transform (LioCloud is header-only)

# obstacle_grid_publisher — bridge the grid/costmap/path to rviz2 over DDS (OccupancyGrid /
# PoseStamped / PointCloud2). Visualization only; the planner reads the structs directly.
add_library(obstacle_grid_publisher STATIC src/mapping/obstacle_grid_publisher.cpp)
target_include_directories(obstacle_grid_publisher PUBLIC include)
target_link_libraries(obstacle_grid_publisher PUBLIC route_planner unitree_sdk2)

# goal_receiver — rviz "2D Goal Pose" subscriber (rt/goal_pose).
add_library(goal_receiver STATIC src/planning/goal_receiver.cpp)
target_include_directories(goal_receiver PUBLIC include)
target_link_libraries(goal_receiver PUBLIC unitree_sdk2)

# system/ — the deployment facade: owns the readers/producer + RoutePlanner + publisher, and
# runs the perception / planner / viz threads. The kist-navigation-planner binary runs this.
add_library(nav_system STATIC src/system/nav_system.cpp)
target_include_directories(nav_system PUBLIC include)
target_link_libraries(nav_system PUBLIC
    route_planner goal_receiver obstacle_grid_publisher lio_receiver lio_transform_producer
    unitree_state_reader yaml-cpp)

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

# ─── LIO engine: external DDS service (not built here) ────────────────────────
# LIO now runs as a separate ROS2 process (deepglint FAST_LIO_LOCALIZATION_HUMANOID,
# raw Livox driver), pinned by the docker image — see docs/LIO_ENGINE.md. It talks to
# this ROS-free planner ONLY over DDS: it publishes T_odom_lidar on /Odometry_loc
# (nav_msgs/Odometry, frame camera_init) and the registered cloud on /cloud_registered_1
# (sensor_msgs/PointCloud2, odom frame). The planner subscribes to those. Our earlier
# ROS-free FAST-LIO port (src/lio/, ikd-Tree, probes) is retired under _parked/.
