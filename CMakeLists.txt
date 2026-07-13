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

# ─── unitree_pointcloud_reader ────────────────────────────────────────────────
add_library(unitree_pointcloud_reader STATIC
    src/unitree/unitree_pointcloud_reader.cpp
    src/unitree/pointcloud2_decode.cpp
)
target_include_directories(unitree_pointcloud_reader PUBLIC include)
target_link_libraries(unitree_pointcloud_reader PUBLIC unitree_sdk2)

# ─── tests ────────────────────────────────────────────────────────────────────
add_executable(test_pointcloud_decode test/test_pointcloud_decode.cpp)
target_link_libraries(test_pointcloud_decode PRIVATE unitree_pointcloud_reader)

add_executable(test_unitree_pointcloud_reader test/test_unitree_pointcloud_reader.cpp)
target_link_libraries(test_unitree_pointcloud_reader PRIVATE unitree_pointcloud_reader)
