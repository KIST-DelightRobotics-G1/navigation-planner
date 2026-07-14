cmake_minimum_required(VERSION 3.18)
project(kist_navigation_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)

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

# ─── tests ────────────────────────────────────────────────────────────────────
add_executable(test_pointcloud2_decode test/test_pointcloud2_decode.cpp)
target_link_libraries(test_pointcloud2_decode PRIVATE unitree_pointcloud_reader)

add_executable(test_pointcloud_processor test/test_pointcloud_processor.cpp)
target_link_libraries(test_pointcloud_processor PRIVATE pointcloud_processing)

add_executable(test_unitree_pointcloud_reader test/test_unitree_pointcloud_reader.cpp)
target_link_libraries(test_unitree_pointcloud_reader
    PRIVATE unitree_pointcloud_reader pointcloud_processing)

add_executable(test_odometry_convert test/test_odometry_convert.cpp)
target_link_libraries(test_odometry_convert PRIVATE unitree_odometry_reader)

add_executable(test_unitree_odometry_reader test/test_unitree_odometry_reader.cpp)
target_link_libraries(test_unitree_odometry_reader
    PRIVATE unitree_odometry_reader yaml-cpp)
