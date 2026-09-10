# kist-navigation-planner — single all-in-one image: the ROS-free planner PLUS the
# LIO engine it consumes over DDS (docs/LIO_ENGINE.md).
#
#   [this image]
#     /opt/lio_ws          LIO engine (ROS2): deepglint FAST-LIO + Livox driver
#     /workspace/kist-...  the ROS-free planner (built with plain cmake)
#   The two run as separate processes and talk over DDS on localhost (RTPS interop,
#   validated FastDDS<->CycloneDDS). The engine is orchestrated by our own launch
#   (lio_engine/lio_bringup.launch.py) — upstream source is cloned at build time and
#   built, never vendored into the repo.
#
# Base is ros:humble (Ubuntu 22.04 + ROS2 Humble) so the engine's ROS2 nodes build/run;
# the planner stays ROS-free (plain cmake, does not use ament/rclcpp).
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# ─── build tools + deps (planner: yaml-cpp/eigen; engine: pcl + ros pcl_ros) ───
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates \
    libyaml-cpp-dev libeigen3-dev \
    libpcl-dev ros-humble-pcl-ros \
    ros-humble-rviz2 \
    iputils-ping nano vim less gdb \
    && rm -rf /var/lib/apt/lists/*

# ─── LIO engine dep: Livox-SDK2 (UDP driver core) ─────────────────────────────
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/livox-sdk2 && \
    cmake -S /tmp/livox-sdk2 -B /tmp/livox-sdk2/build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/livox-sdk2/build --target install -j"$(nproc)" && \
    rm -rf /tmp/livox-sdk2

# ─── LIO engine: deepglint FAST-LIO (humble) + modified livox_ros_driver2 (main) ─
# The working setup spans two branches (docs/LIO_ENGINE.md): the ROS2 FAST_LIO lives on
# `humble`, the IMU-roll-fixed driver on `main`. Clone once, copy each package from its
# pinned commit into one colcon workspace, then build (driver first — FAST_LIO needs its
# CustomMsg). The 640 MB clone is build-time only; nothing is vendored into the repo.
ARG DEEPGLINT_FASTLIO_COMMIT=df4772ec4797172430e7efe990711d09a529f4ad
ARG DEEPGLINT_DRIVER_COMMIT=e4234d4e559548bfe0afdf407990bfb206e183c0
RUN mkdir -p /opt/lio_ws/src && \
    git clone https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git /tmp/dg && \
    git -C /tmp/dg checkout "${DEEPGLINT_DRIVER_COMMIT}" && \
    cp -r /tmp/dg/livox_ros_driver2 /opt/lio_ws/src/ && \
    git -C /tmp/dg checkout "${DEEPGLINT_FASTLIO_COMMIT}" && \
    cp -r /tmp/dg/FAST_LIO /opt/lio_ws/src/fast_lio && \
    rm -rf /tmp/dg && \
    bash -c 'source /opt/ros/humble/setup.bash && cd /opt/lio_ws && \
             bash ./src/livox_ros_driver2/build.sh humble && \
             source install/setup.bash && \
             colcon build --packages-select fast_lio'

# ─── planner: vendored unitree_sdk2 (CycloneDDS + ROS2 IDL types) ──────────────
WORKDIR /workspace/kist-navigation-planner
ARG UNITREE_SDK2_COMMIT=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2 && \
    git -C thirdparty/unitree_sdk2 checkout "${UNITREE_SDK2_COMMIT}"

# ─── idlc + idlcxx 0.10.2 (only for goal_generation's custom kist_nav.idl) ─────
# Isolated in /opt so it never shadows unitree_sdk2's runtime libs or ROS's cyclonedds.
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

# ─── planner: source + build (plain cmake, ROS-free) ──────────────────────────
# Build context = repo root minus build/, .git, the SDK, and _parked/ (.dockerignore).
COPY . .
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j"$(nproc)"

# entrypoint sources ROS + the LIO engine overlay so `ros2 launch lio_engine/...` and
# the planner binaries both work in an interactive shell.
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
