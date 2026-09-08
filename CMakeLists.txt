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

# Articulated-body forward kinematics: joint state q(t) -> dynamic transforms,
# plus the thin waist->tree producer.
add_library(kinematics STATIC
    src/kinematics/g1_kinematics.cpp
    src/kinematics/waist_producer.cpp)
target_include_directories(kinematics PUBLIC include)
target_link_libraries(kinematics PUBLIC transforms)

# Transform SE(3) math + pose interpolation — pin the conventions with tests.
add_executable(test_transform test/test_transform.cpp)
target_include_directories(test_transform PRIVATE include)
target_link_libraries(test_transform PRIVATE Eigen3::Eigen)

add_executable(test_pose_history test/test_pose_history.cpp)
target_link_libraries(test_pose_history PRIVATE transforms)

# Frame tree — static + dynamic edge composition, time interpolation, deskew lookup.
add_executable(test_transform_tree test/test_transform_tree.cpp)
target_link_libraries(test_transform_tree PRIVATE transforms)

# G1 FK (Transform) vs legacy Rigid cross-check + waist producer/tree integration.
add_executable(test_g1_kinematics test/test_g1_kinematics.cpp)
target_link_libraries(test_g1_kinematics PRIVATE kinematics)

# G1 leg FK (pelvis -> ankle_roll) — q=0 identity, geometry, mirror symmetry.
add_executable(test_g1_foot_fk test/test_g1_foot_fk.cpp)
target_link_libraries(test_g1_foot_fk PRIVATE kinematics)

# ─── thirdparty: invariant-ekf (contact-aided InEKF) ──────────────────────────
# Upstream CMakeLists forces the compiler + global flags + builds Boost examples,
# so we compile only the four CORE sources into our own target (no add_subdirectory,
# no pollution). Core needs Eigen only — Boost is used solely by the examples.
add_library(inekf STATIC
    thirdparty/invariant-ekf/src/InEKF.cpp
    thirdparty/invariant-ekf/src/RobotState.cpp
    thirdparty/invariant-ekf/src/LieGroup.cpp
    thirdparty/invariant-ekf/src/NoiseParams.cpp)
target_include_directories(inekf PUBLIC thirdparty/invariant-ekf/include)
target_link_libraries(inekf PUBLIC Eigen3::Eigen)
target_compile_definitions(inekf PUBLIC INEKF_USE_MUTEX=false)
target_compile_features(inekf PUBLIC cxx_std_17)

# Smoke test: the vendored lib compiles/links with Eigen alone and the API matches.
add_executable(test_inekf_link test/test_inekf_link.cpp)
target_link_libraries(test_inekf_link PRIVATE inekf)

# odom->pelvis estimator: InEKF wrapper hiding invariant-ekf behind our own types.
add_library(kist_state_estimation STATIC src/state_estimation/g1_state_estimator.cpp)
target_include_directories(kist_state_estimation PUBLIC include)
target_link_libraries(kist_state_estimation PUBLIC kinematics Eigen3::Eigen PRIVATE inekf)

add_executable(test_g1_state_estimator test/test_g1_state_estimator.cpp)
target_link_libraries(test_g1_state_estimator PRIVATE kist_state_estimation)

# Mid-360 IMU discovery probe — find the DDS topic that relays the LiDAR's IMU.
add_executable(test_lidar_imu_probe test/test_lidar_imu_probe.cpp)
target_include_directories(test_lidar_imu_probe PRIVATE include)
target_link_libraries(test_lidar_imu_probe PRIVATE unitree_sdk2 yaml-cpp)

# Static-extrinsics loader — YAML load, name->FrameId, RPY->quat, rejections.
add_executable(test_sensor_extrinsics test/test_sensor_extrinsics.cpp)
target_link_libraries(test_sensor_extrinsics PRIVATE frames)
