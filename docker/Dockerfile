# kist-navigation-planner — build image for the ROS-free navigation foundation.
#
# Slimmed to the current build: the frame / transform / kinematics / state-
# estimation spine + unitree DDS readers + goal_generation. The GPU CV-inference
# and camera-receive stacks are parked (see _parked/), so CUDA / TensorRT /
# OpenCV / libav are no longer installed and the base is plain Ubuntu.
#
# Self-contained: unitree_sdk2 is cloned (pinned) in-image, CycloneDDS idlc is
# built for goal_generation's custom DDS types, then the repo is COPY'd and built.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ─── build tools + deps ─────────────────────────────────────────────────────
#   libyaml-cpp-dev : config parsing
#   libeigen3-dev   : transform / estimation linear algebra
#   libpcl-dev      : FAST-LIO point clouds + ikd-Tree (heavy: pulls boost/flann/vtk)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ca-certificates \
    libyaml-cpp-dev \
    libeigen3-dev \
    libpcl-dev \
    nano vim less gdb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/kist-navigation-planner

# ─── vendored dep: unitree_sdk2 (cloned + pinned) ─────────────────────────────
# Bundles CycloneDDS + the ROS2 IDL types (PointCloud2_, Imu_, Odometry_,
# LowState) the readers use. Cloned before COPY so source edits don't re-clone.
ARG UNITREE_SDK2_COMMIT=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2 && \
    git -C thirdparty/unitree_sdk2 checkout "${UNITREE_SDK2_COMMIT}"

# ─── idlc + idlcxx 0.10.2 (only for goal_generation's custom kist_nav.idl) ─────
# Matches unitree_sdk2's bundled libddscxx. Installed in /opt so it never shadows
# the SDK's runtime libs — only the build-time tool is used. Without it, CMake
# skips the goal_generation DDS layer and the rest of the repo still builds.
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
# DDS transport config (NIC + rx socket tuning). apply_dds_config() honors this.
ENV CYCLONEDDS_URI=file:///workspace/kist-navigation-planner/config/cyclonedds.xml

# ─── source + build ───────────────────────────────────────────────────────────
# Build context = repo root minus build/, .git, the SDK, and _parked/ (.dockerignore).
COPY . .
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j"$(nproc)"

CMD ["/bin/bash"]
