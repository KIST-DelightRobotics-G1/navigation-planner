# kist-navigation-planner — build image for the GPU CV-inference path.
#
# Base is TensorRT 10.7 + CUDA 12.6 (matches kist-gearsonic-inference and the
# Jetson JetPack 6 target). The vendored tensorrt/InferenceEngine targets TRT
# 10.7, so it only compiles here — NOT against a host's differing TRT.
#
# Self-contained like ext-sensor-io: unitree_sdk2 is cloned (pinned) in-image,
# the repo is COPY'd in, and everything builds at image-build time.
#
# Only the essentials are installed. Deliberately NOT here (added when needed):
#   - idlc / CycloneDDS-CXX  : no custom DDS IDL (we use the SDK's ROS2 types)
#   - librealsense2/x264/libav: no camera capture/codec in-repo (live-camera
#                               fusion via ext-sensor-io's RealsenseReceiver is
#                               a later layer: needs libav + idlc + that repo)
FROM nvcr.io/nvidia/tensorrt:24.12-py3

ENV DEBIAN_FRONTEND=noninteractive

# ─── build tools + deps ─────────────────────────────────────────────────────
#   libyaml-cpp-dev : config parsing
#   libeigen3-dev   : EKF linear algebra (pose_filter)
#   libopencv-dev   : visual test runners (pose/seg viewers)
# CUDA + TensorRT (nvinfer/nvonnxparser) come from the base image.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ca-certificates \
    libyaml-cpp-dev \
    libeigen3-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/kist-navigation-planner

# ─── vendored dep: unitree_sdk2 (cloned + pinned) ─────────────────────────────
# Bundles CycloneDDS + the ROS2 IDL types (PointCloud2_, Odometry_) the readers
# use — no idlc needed. Cloned before COPY so source edits don't re-clone.
ARG UNITREE_SDK2_COMMIT=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2 && \
    git -C thirdparty/unitree_sdk2 checkout "${UNITREE_SDK2_COMMIT}"

# ─── source + build ───────────────────────────────────────────────────────────
# Build context = repo root (minus build/, .git, the SDK via .dockerignore).
COPY . .
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j"$(nproc)"

CMD ["/bin/bash"]
