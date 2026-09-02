cmake_minimum_required(VERSION 3.18)
project(kist_navigation_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)       # header-only; EKF linear algebra
find_package(CUDAToolkit REQUIRED)  # CV inference (TensorRT backend)
find_package(OpenCV REQUIRED)       # CV inference pre/post + visual runners

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

# ─── tensorrt_inference ───────────────────────────────────────────────────────
# Generic ONNX->TensorRT engine backend (build + cache + execute), vendored
# as-is from kist-gearsonic-inference — reused for the CV segmentation model
# (model-agnostic named multi-I/O, so YOLO-seg's 1-in/2-out fits). Same CMake
# as upstream: unconditional, C++20, links nvinfer/nvonnxparser by name.
#
# The code targets TensorRT 10.7 (nvcr.io/nvidia/tensorrt:24.12-py3, matching
# the Jetson JetPack 6 target) — build it in that container, like gearsonic and
# ext-sensor-io. A host with a different TRT will fail to compile here (by
# design: a loud "wrong toolchain, use the Docker" error, not a silent skip).
add_library(tensorrt_inference STATIC
    src/tensorrt/InferenceEngine.cpp
)
target_include_directories(tensorrt_inference PUBLIC
    include
)
target_include_directories(tensorrt_inference PRIVATE
    thirdparty/picosha2
)
target_link_libraries(tensorrt_inference PUBLIC
    CUDA::cudart
    nvinfer
    nvonnxparser
)
set_target_properties(tensorrt_inference PROPERTIES CXX_STANDARD 20)

# ─── cv_yolo ──────────────────────────────────────────────────────────────────
# CV YOLO inference under cv/yolo/. Shared generics (yolo_preprocess, yolo_inference,
# and the header-only templated YoloPipeline worker) + two task engines:
# instance_segmentation/ (proto-mask postprocess, cuBLAS GEMM) and
# semantic_segmentation/ (dense UINT8 class map, trivial postprocess). Pipeline:
# letterbox preprocess -> TRT (tensorrt_inference) -> task postprocess ->
# DataBuffer<Result>. C++20 (pulls the TRT engine header). YoloPipeline is a
# header-only template, so nothing to compile for it here.
add_library(cv_yolo STATIC
    src/cv/yolo/yolo_preprocess.cpp
    src/cv/yolo/yolo_inference.cpp
    src/cv/yolo/instance_segmentation/yolo_inst_seg_postprocess.cpp
    src/cv/yolo/instance_segmentation/yolo_inst_seg_engine.cpp
    src/cv/yolo/semantic_segmentation/yolo_sem_seg_postprocess.cpp
    src/cv/yolo/semantic_segmentation/yolo_sem_seg_engine.cpp
)
target_include_directories(cv_yolo PUBLIC include ${OpenCV_INCLUDE_DIRS})
# CUDA::cublas + cudart for the instance GPU postprocess (coeff x proto GEMM).
target_link_libraries(cv_yolo PUBLIC tensorrt_inference ${OpenCV_LIBS}
    CUDA::cublas CUDA::cudart)
set_target_properties(cv_yolo PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)

# Static-image instance-seg check — the model goes in models/ (yolo26l-seg.onnx).
add_executable(test_yolo_seg test/test_yolo_seg.cpp)
target_link_libraries(test_yolo_seg PRIVATE cv_yolo)
set_target_properties(test_yolo_seg PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)

# ─── tests ────────────────────────────────────────────────────────────────────
# Live runners only — they subscribe to the robot's DDS streams. The pose_filter
# / pointcloud_processing / unitree_* libraries are exercised through these.
add_executable(test_unitree_pointcloud_reader test/test_unitree_pointcloud_reader.cpp)
target_link_libraries(test_unitree_pointcloud_reader
    PRIVATE unitree_pointcloud_reader pointcloud_processing)

# Visual variant (OpenCV): top-down bird's-eye projection of the LiDAR cloud,
# colored by height. Window when DISPLAY is set, else /tmp/pointcloud_view.png.
add_executable(test_pointcloud_viewer test/test_pointcloud_viewer.cpp)
target_link_libraries(test_pointcloud_viewer
    PRIVATE unitree_pointcloud_reader pointcloud_processing ${OpenCV_LIBS})
target_include_directories(test_pointcloud_viewer PRIVATE ${OpenCV_INCLUDE_DIRS})

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

    # Live camera CV: RealsenseReceiver (H.264 color, embedded) -> YoloPipeline
    # -> OpenCV overlay. Instance-seg (masks) and semantic-seg (class map) share
    # the receiver wiring. ext's realsense_receiver only exists when its Rx deps
    # (libav + idlc) are present — i.e. the Docker image.
    if(TARGET realsense_receiver AND OpenCV_FOUND)
        add_executable(test_seg_camera test/test_seg_camera.cpp)
        target_link_libraries(test_seg_camera
            PRIVATE cv_yolo realsense_receiver yaml-cpp ${OpenCV_LIBS})
        set_target_properties(test_seg_camera PROPERTIES
            CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)

        add_executable(test_sem_camera test/test_sem_camera.cpp)
        target_link_libraries(test_sem_camera
            PRIVATE cv_yolo realsense_receiver yaml-cpp ${OpenCV_LIBS})
        set_target_properties(test_sem_camera PROPERTIES
            CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)

        # Labeled-cloud generator: deproject depth + attach the semantic class +
        # transform to the robot frame, on a worker thread. Needs ext's DepthFrame
        # header (via realsense_receiver) so it lives inside this ext-gated block.
        add_library(labeled_cloud_generator STATIC
            src/labeled_cloud/labeled_cloud.cpp
            src/labeled_cloud/camera_extrinsics.cpp
            src/labeled_cloud/labeled_cloud_generator.cpp)
        target_include_directories(labeled_cloud_generator PUBLIC include ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(labeled_cloud_generator PUBLIC realsense_receiver ${OpenCV_LIBS})
        set_target_properties(labeled_cloud_generator PROPERTIES
            CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)

        # Depth+semantic fusion viewer: one receiver -> YOLO class map + the
        # labeled-cloud worker -> robot-frame labeled cloud, shown top-down/side.
        add_executable(test_fusion_viewer test/test_fusion_viewer.cpp)
        target_link_libraries(test_fusion_viewer
            PRIVATE labeled_cloud_generator cv_yolo realsense_receiver yaml-cpp ${OpenCV_LIBS})
        set_target_properties(test_fusion_viewer PROPERTIES
            CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)

        # Occupancy grid: fuse the LiDAR cloud + the labeled camera cloud into a
        # probabilistic semantic grid (occupancy_grid.hpp). Depends on both the
        # labeled-cloud and the LiDAR reader libs, so it lives here too.
        add_library(occupancy_grid STATIC
            src/occupancy_grid/occupancy_grid.cpp
            src/occupancy_grid/occupancy_grid_builder.cpp)
        target_include_directories(occupancy_grid PUBLIC include ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(occupancy_grid
            PUBLIC labeled_cloud_generator unitree_pointcloud_reader)
        set_target_properties(occupancy_grid PROPERTIES
            CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)

        # Full-stack viewer: camera(YOLO+depth) + LiDAR -> fused occupancy grid.
        add_executable(test_occupancy_grid_viewer test/test_occupancy_grid_viewer.cpp)
        target_link_libraries(test_occupancy_grid_viewer
            PRIVATE occupancy_grid labeled_cloud_generator cv_yolo realsense_receiver
                    unitree_pointcloud_reader unitree_odometry_reader pointcloud_processing
                    pose_filter uwb_receiver yaml-cpp ${OpenCV_LIBS})
        set_target_properties(test_occupancy_grid_viewer PROPERTIES
            CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)
        # One-shot frame grab from the camera (clean frame -> disk) for model
        # spikes / offline debugging.
        add_executable(test_camera_snapshot test/test_camera_snapshot.cpp)
        target_link_libraries(test_camera_snapshot
            PRIVATE realsense_receiver yaml-cpp ${OpenCV_LIBS})
        target_include_directories(test_camera_snapshot PRIVATE ${OpenCV_INCLUDE_DIRS})
    else()
        message(STATUS "realsense_receiver/OpenCV missing — skipping test_seg_camera")
    endif()

    message(STATUS "UWB input enabled via ${KIST_EXT_SENSOR_IO}")
else()
    message(STATUS "kist-ext-sensor-io not found alongside this repo "
                   "(${KIST_EXT_SENSOR_IO}) — UWB input disabled")
endif()
