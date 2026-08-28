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

# ─── camera-receive deps (embedding ext-sensor-io's RealsenseReceiver) ────────
# The live-camera seg stage embeds RealsenseReceiver from ext-sensor-io. Its Rx
# build needs libav (H.264 color decode) + the idlc toolchain (CycloneDDS-CXX
# 0.10.2, for the kist_idl custom DDS types — NOT librealsense/x264, those are
# Tx-only). ext-sensor-io itself is a private repo, so it is mounted at run time
# as the sibling ../kist-ext-sensor-io (see run.sh), not cloned here.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libavcodec-dev libavutil-dev libswscale-dev \
    && rm -rf /var/lib/apt/lists/*

# idlc + idlcxx pinned to 0.10.2 (matches unitree_sdk2's bundled libddscxx), in
# /opt so it never shadows the SDK's runtime libs — only the build tools are used.
ARG CYCLONEDDS_VERSION=0.10.2
RUN git clone --depth 1 --branch "${CYCLONEDDS_VERSION}" \
        https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/cyclonedds && \
    cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build \
        -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds \
        -DBUILD_IDLC=ON -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/cyclonedds/build --target install -j "$(nproc)" && \
    git clone --depth 1 --branch "${CYCLONEDDS_VERSION}" \
        https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git /tmp/cyclonedds-cxx && \
    cmake -S /tmp/cyclonedds-cxx -B /tmp/cyclonedds-cxx/build \
        -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds \
        -DCMAKE_PREFIX_PATH=/opt/cyclonedds -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/cyclonedds-cxx/build --target install -j "$(nproc)" && \
    rm -rf /tmp/cyclonedds /tmp/cyclonedds-cxx
ENV PATH=/opt/cyclonedds/bin:${PATH}
ENV CYCLONEDDS_HOME=/opt/cyclonedds
# Route the DDS transport config (NIC + rx socket/defrag tuning) so the camera
# receiver doesn't drop frames. apply_dds_config() honors this pre-set value.
# See config/cyclonedds.xml re: the host net.core.rmem_max cap under --network host.
ENV CYCLONEDDS_URI=file:///workspace/kist-navigation-planner/config/cyclonedds.xml

# ─── source + build ───────────────────────────────────────────────────────────
# Build context = repo root (minus build/, .git, the SDK via .dockerignore).
# The camera stage (ext-sensor-io mount) is absent at image-build time, so it is
# skipped here and built in-container after run.sh mounts the sibling repo.
COPY . .
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j"$(nproc)"

CMD ["/bin/bash"]
