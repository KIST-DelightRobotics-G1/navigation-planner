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

# ─── unitree_pointcloud_reader ────────────────────────────────────────────────
# LiDAR frames straight from the robot's DDS relay (rt/utlidar/...);
# decode → optional processor hook → DataBuffer, all on the DDS
# receive thread.
add_library(unitree_pointcloud_reader STATIC
    src/unitree/unitree_pointcloud_reader.cpp
)
target_include_directories(unitree_pointcloud_reader PUBLIC include)
target_link_libraries(unitree_pointcloud_reader PUBLIC unitree_sdk2)

# ─── unitree_pointcloud_imu_reader ────────────────────────────────────────────
# Mid-360 built-in IMU (rt/utlidar/imu_livox_mid360, ~200Hz) -> ImuSample hand-off
# queue for the LIO worker. Separate subscriber/thread from the cloud reader.
add_library(unitree_pointcloud_imu_reader STATIC
    src/unitree/unitree_pointcloud_imu_reader.cpp
)
target_include_directories(unitree_pointcloud_imu_reader PUBLIC include)
target_link_libraries(unitree_pointcloud_imu_reader PUBLIC unitree_sdk2 Eigen3::Eigen)

# ─── unitree_odometry_reader ──────────────────────────────────────────────────
# Raw locomotion-controller odometry (rt/dog_odom); odom-frame, drifts —
# the EKF fusion stage (odom predict + UWB update) owns global pose.
add_library(unitree_odometry_reader STATIC
    src/unitree/unitree_odometry_reader.cpp
)
target_include_directories(unitree_odometry_reader PUBLIC include)
target_link_libraries(unitree_odometry_reader PUBLIC unitree_sdk2)

# ─── g1_lowstate_reader ───────────────────────────────────────────────────────
# G1 lowstate (rt/lowstate) -> waist joints, for FK sensor-bob compensation.
add_library(g1_lowstate_reader STATIC
    src/unitree/g1_lowstate_reader.cpp
)
target_include_directories(g1_lowstate_reader PUBLIC include)
target_link_libraries(g1_lowstate_reader PUBLIC unitree_sdk2)

add_executable(test_lowstate_probe test/test_lowstate_probe.cpp)
target_link_libraries(test_lowstate_probe PRIVATE g1_lowstate_reader yaml-cpp)

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

# kinematics/ (waist+foot FK) + state_estimation/ (InEKF) are PARKED under _parked/:
# LIO estimates T_odom_lidar directly, so waist FK isn't needed on this branch.
# The invariant-ekf submodule stays in thirdparty/ (unbuilt).

# Transform SE(3) math + pose interpolation — pin the conventions with tests.
add_executable(test_transform test/test_transform.cpp)
target_include_directories(test_transform PRIVATE include)
target_link_libraries(test_transform PRIVATE Eigen3::Eigen)

add_executable(test_pose_history test/test_pose_history.cpp)
target_link_libraries(test_pose_history PRIVATE transforms)

# Frame tree — static + dynamic edge composition, time interpolation, deskew lookup.
add_executable(test_transform_tree test/test_transform_tree.cpp)
target_link_libraries(test_transform_tree PRIVATE transforms)

# Mid-360 IMU discovery probe — find the DDS topic that relays the LiDAR's IMU.
add_executable(test_lidar_imu_probe test/test_lidar_imu_probe.cpp)
target_include_directories(test_lidar_imu_probe PRIVATE include)
target_link_libraries(test_lidar_imu_probe PRIVATE unitree_sdk2 yaml-cpp)

# LiDAR cloud field inspector — check for a per-point timestamp channel (deskew).
add_executable(test_lidar_cloud_fields test/test_lidar_cloud_fields.cpp)
target_include_directories(test_lidar_cloud_fields PRIVATE include)
target_link_libraries(test_lidar_cloud_fields PRIVATE unitree_sdk2 yaml-cpp)

# Mid-360 IMU reader test — drain the hand-off queue, verify rate + g->m/s^2.
add_executable(test_lidar_imu_reader test/test_lidar_imu_reader.cpp)
target_link_libraries(test_lidar_imu_reader PRIVATE unitree_pointcloud_imu_reader yaml-cpp)

# Cloud<->IMU frame check — does the cloud's floor agree with IMU gravity direction?
add_executable(test_frame_check test/test_frame_check.cpp)
target_include_directories(test_frame_check PRIVATE include)
target_link_libraries(test_frame_check PRIVATE unitree_sdk2 yaml-cpp)

# Static-extrinsics loader — YAML load, name->FrameId, RPY->quat, rejections.
add_executable(test_sensor_extrinsics test/test_sensor_extrinsics.cpp)
target_link_libraries(test_sensor_extrinsics PRIVATE frames)

# ─── thirdparty: FAST-LIO2 (LIO estimator) ────────────────────────────────────
# FAST-LIO is a ROS1 catkin package, NOT a library, so we do not add_subdirectory
# it. Its PURE core (IKFoM ESIKF: use-ikfom / IKFoM_toolkit / so3_math) is ROS- and
# PCL-free; ikd-Tree needs PCL only. The ROS-coupled parts (common_lib /
# IMU_Processing / laserMapping) get ported into our own src/lio/ (copied + de-ROS'd)
# in later stages. This block builds ikd-Tree + a smoke test to prove the pure core
# and PCL compile ROS-free in this image before that port.
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)   # FAST-LIO's IKFoM/laserMapping use omp_get_wtime + parallel for
set(FASTLIO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/fast-lio)

add_library(ikd_tree STATIC ${FASTLIO_DIR}/include/ikd-Tree/ikd_Tree.cpp)
target_include_directories(ikd_tree PUBLIC ${FASTLIO_DIR}/include ${PCL_INCLUDE_DIRS})
target_link_libraries(ikd_tree PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen)

# Smoke: IKFoM state + ikd-Tree + PCL build/link with no ROS in this image.
# BOOST_BIND_GLOBAL_PLACEHOLDERS silences boost/bind's deprecation pragma pulled in
# by IKFoM's esekfom.hpp.
add_executable(test_fastlio_core test/test_fastlio_core.cpp)
target_include_directories(test_fastlio_core PRIVATE include ${FASTLIO_DIR}/include ${PCL_INCLUDE_DIRS})
target_compile_definitions(test_fastlio_core PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
target_link_libraries(test_fastlio_core PRIVATE ikd_tree ${PCL_LIBRARIES} Eigen3::Eigen OpenMP::OpenMP_CXX)

# The LIO worker: FAST-LIO2 ESIKF + ikd-Tree core, ported ROS-free behind a PIMPL
# facade (only Eigen + our types leak). PCL / FAST-LIO includes stay PRIVATE.
add_library(lio STATIC src/lio/lio_worker.cpp)
target_include_directories(lio PUBLIC include PRIVATE ${FASTLIO_DIR}/include ${PCL_INCLUDE_DIRS})
target_compile_definitions(lio PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
target_link_libraries(lio PUBLIC Eigen3::Eigen PRIVATE ikd_tree ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

# LioWorker smoke — synthetic frames through the full worker (no PCL in this TU).
add_executable(test_lio_worker test/test_lio_worker.cpp)
target_link_libraries(test_lio_worker PRIVATE lio)

# LIO runtime: live DDS Mid-360 (cloud + IMU readers) -> sync -> worker -> T_odom_lidar.
add_library(lio_runtime STATIC src/lio/lio_runtime.cpp)
target_include_directories(lio_runtime PUBLIC include)
target_link_libraries(lio_runtime PUBLIC lio unitree_pointcloud_reader unitree_pointcloud_imu_reader)

# Live LIO runner — first real-data run; prints T_odom_lidar at ~5 Hz.
add_executable(test_lio_run test/test_lio_run.cpp)
target_link_libraries(test_lio_run PRIVATE lio_runtime yaml-cpp)
