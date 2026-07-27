cmake_minimum_required(VERSION 3.18)
project(kist_navigation_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)  # header-only; EKF linear algebra

# ─── thirdparty: unitree_sdk2 ─────────────────────────────────────────────────
# Guarded so an embedding super-project that already provides the target
# (e.g. via kist-gearsonic-inference) composes without a collision.
if(NOT TARGET unitree_sdk2)
    set(BUILD_EXAMPLES OFF CACHE BOOL "Build unitree_sdk2 examples" FORCE)
    add_subdirectory(thirdparty/unitree_sdk2)
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

# ─── pointcloud_processing ────────────────────────────────────────────────────
# Processing stage (transform + filter, configured from config.yaml);
# owns no thread — plugged into the reader via set_processor.
add_library(pointcloud_processing STATIC
    src/pointcloud/pointcloud_processor.cpp
)
target_include_directories(pointcloud_processing PUBLIC include)
target_link_libraries(pointcloud_processing PUBLIC yaml-cpp)

# ─── pose_filter ──────────────────────────────────────────────────────────────
# UWB + odometry fusion (adaptive EKF with online yaw-bias, ported from the
# onboard UwbOdomAEKF). PoseFilter is a Thread that owns the EKF and reads/writes
# via DataBuffers: pulls odom from an injected buffer, takes UWB fixes on uwb_buf,
# publishes pose_buf. yaml-free (config in runners); depends only on the
# UnitreeOdometry struct header (include/) + Eigen.
add_library(pose_filter STATIC
    src/kalman_filter/uwb_odom_aekf.cpp
    src/kalman_filter/pose_filter.cpp
)
target_include_directories(pose_filter PUBLIC include)
target_link_libraries(pose_filter PUBLIC Eigen3::Eigen yaml-cpp)

# ─── tests ────────────────────────────────────────────────────────────────────
# Live runners only — they subscribe to the robot's DDS streams. The pose_filter
# / pointcloud_processing / unitree_* libraries are exercised through these.
add_executable(test_unitree_pointcloud_reader test/test_unitree_pointcloud_reader.cpp)
target_link_libraries(test_unitree_pointcloud_reader
    PRIVATE unitree_pointcloud_reader pointcloud_processing)

add_executable(test_unitree_odometry_reader test/test_unitree_odometry_reader.cpp)
target_link_libraries(test_unitree_odometry_reader
    PRIVATE unitree_odometry_reader yaml-cpp)

# ─── optional: UWB pose input (embedded from kist-ext-sensor-io) ──────────────
# UWB global-position fixes come from the sibling kist-ext-sensor-io repo, which
# is NOT vendored here — reference it in place when its checkout sits alongside
# this one. EXCLUDE_FROM_ALL keeps ext's other targets (realsense, idlc) from
# building; only uwb_receiver is pulled in on demand by the runner below.
# Absent → skipped, and this repo still builds standalone.
#
# Both repos ship an identical common/ (data_buffer.hpp, config.hpp) — the
# duplicate include roots resolve to the same definitions today, but keep the
# two copies in sync (a divergence would silently pick one by -I order).
set(KIST_EXT_SENSOR_IO "${CMAKE_CURRENT_SOURCE_DIR}/../kist-ext-sensor-io"
    CACHE PATH "Path to the kist-ext-sensor-io checkout (UWB pose input)")
if(EXISTS "${KIST_EXT_SENSOR_IO}/CMakeLists.txt")
    # unitree_sdk2 is added above, so ext's guarded add_subdirectory skips its
    # own copy and both link the same target.
    add_subdirectory("${KIST_EXT_SENSOR_IO}" ext_sensor_io EXCLUDE_FROM_ALL)
    add_executable(test_uwb_reader test/test_uwb_reader.cpp)
    target_link_libraries(test_uwb_reader PRIVATE uwb_receiver yaml-cpp)

    # Live end-to-end fusion: odom reader + UwbReceiver -> PoseFilter.
    add_executable(test_pose_filter test/test_pose_filter.cpp)
    target_link_libraries(test_pose_filter
        PRIVATE pose_filter unitree_odometry_reader uwb_receiver yaml-cpp)

    # Visual variant (OpenCV): blue dot = fused pose, blue arrow = yaw,
    # red X = raw UWB. Values still logged. Window when DISPLAY is set,
    # else periodic /tmp/pose_filter_view.png. Skipped when OpenCV is absent.
    find_package(OpenCV QUIET)
    if(OpenCV_FOUND)
        add_executable(test_pose_filter_viewer test/test_pose_filter_viewer.cpp)
        target_link_libraries(test_pose_filter_viewer
            PRIVATE pose_filter unitree_odometry_reader uwb_receiver yaml-cpp ${OpenCV_LIBS})
        target_include_directories(test_pose_filter_viewer PRIVATE ${OpenCV_INCLUDE_DIRS})
    else()
        message(STATUS "OpenCV not found — skipping test_pose_filter_viewer")
    endif()

    message(STATUS "UWB input enabled via ${KIST_EXT_SENSOR_IO}")
else()
    message(STATUS "kist-ext-sensor-io not found alongside this repo "
                   "(${KIST_EXT_SENSOR_IO}) — UWB input disabled")
endif()
