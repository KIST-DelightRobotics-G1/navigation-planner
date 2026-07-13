cmake_minimum_required(VERSION 3.18)
project(kist_navigation_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# NOTE: include/route_planner + src/{astar,costmap,...} are the imported
# legacy pipeline, kept as the algorithm reference while stages are
# reimplemented — they are not built.

# ─── thirdparty: unitree_sdk2 ─────────────────────────────────────────────────
# Guarded so an embedding super-project that already provides the target
# (e.g. via kist-gearsonic-inference) composes without a collision.
if(NOT TARGET unitree_sdk2)
    set(BUILD_EXAMPLES OFF CACHE BOOL "Build unitree_sdk2 examples" FORCE)
    add_subdirectory(thirdparty/unitree_sdk2)
endif()

# ─── lidar_reader ─────────────────────────────────────────────────────────────
add_library(lidar_reader STATIC
    src/lidar/lidar_reader.cpp
    src/lidar/pointcloud2_decode.cpp
)
target_include_directories(lidar_reader PUBLIC include)
target_link_libraries(lidar_reader PUBLIC unitree_sdk2)

# ─── tests ────────────────────────────────────────────────────────────────────
add_executable(test_lidar_decode test/test_lidar_decode.cpp)
target_link_libraries(test_lidar_decode PRIVATE lidar_reader)

add_executable(test_lidar_reader test/test_lidar_reader.cpp)
target_link_libraries(test_lidar_reader PRIVATE lidar_reader)
