# LIO engine (external DDS service)

LIO runs as a **separate ROS2 process**, not linked into this ROS-free planner. It is
the upstream **deepglint FAST_LIO_LOCALIZATION_HUMANOID** stack, driven by the raw
Livox driver, run unmodified. It talks to the planner **only over DDS** (the robot's
DDS is ROS2-compatible), so ROS2 stays isolated inside the engine's container.

Why not our own port: our ROS-free FAST-LIO port (now under `_parked/`) worked but the
**utlidar** relay's cloud data caused straight-line stretch + fast-rotation doubling.
Feeding FAST-LIO the **raw Livox** data instead removes the problem — validated on the
real G1 (stationary stable, ~1.5 m move read as ~1.40 m, walls stay put in rviz).
See memory `lio-utlidar-vs-raw-livox`.

## Pin

| Piece | Source | Commit |
|---|---|---|
| FAST_LIO (ROS2) | deepglint `humble` branch | `df4772ec4797172430e7efe990711d09a529f4ad` |
| modified livox_ros_driver2 | deepglint `main` branch | `e4234d4e559548bfe0afdf407990bfb206e183c0` |

Repo: https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git

The exact working build is captured in the docker image **`lio-humble:snap`**
(`osrf/ros:humble-desktop` + Livox-SDK2 + both packages built in `/root/ws_lio`).
That image is the reproducible pin — the repo is not vendored here (it is ~640 MB).

## How the image was built (to reproduce from scratch)

```bash
# osrf/ros:humble-desktop container, --network host
apt update && apt install -y git cmake build-essential libeigen3-dev libpcl-dev ros-humble-pcl-ros
# Livox-SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git && cd Livox-SDK2 && \
  mkdir build && cd build && cmake .. && make -j && make install
# workspace: driver from main, FAST_LIO from humble
mkdir -p ~/ws_lio/src && cd ~/ws_lio/src
cp -r <deepglint@main>/livox_ros_driver2 .
cp -r <deepglint@humble>/FAST_LIO ./fast_lio
cd ~/ws_lio && bash ./src/livox_ros_driver2/build.sh humble && \
  colcon build --packages-select fast_lio
```

Config already correct for the G1 (in the driver's `config/MID360_config.json`):
`extrinsic_parameter.roll = 180` (upside-down mount), lidar ip `192.168.123.120`,
host_net_info ip `192.168.123.222` (must match the host's robot-LAN IP). In
`fast_lio/config/mid360.yaml`, `lid_topic` must be `/livox/lidar` (not
`/livox/custom_msg`). `mkdir -p ~/ws_lio/src/fast_lio/PCD` if you ever enable PCD save
(it is dead in the humble branch anyway).

## Run

```bash
# HOST — start the engine container (detached, keeps running)
xhost +local:root
docker run -d --name lio --network host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix lio-humble:snap sleep infinity

# Terminal 1 — driver
docker exec -it lio bash -c \
 'source /opt/ros/humble/setup.bash && source /root/ws_lio/install/setup.bash && \
  ros2 launch livox_ros_driver2 msg_MID360_launch.py'

# Terminal 2 — FAST-LIO
docker exec -it lio bash -c \
 'source /opt/ros/humble/setup.bash && source /root/ws_lio/install/setup.bash && \
  ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false'

# (optional) rviz: run `rviz2` standalone in the container; the launch's auto-rviz
# does not open in this container, so add displays by hand — Fixed Frame camera_init,
# PointCloud2 on /cloud_registered_1 (raise Decay Time), Odometry on /Odometry_loc.
```

Prereq: host is on the robot LAN (`192.168.123.222`), lidar reachable
(`ping 192.168.123.120`).

## Topic contract (what the planner subscribes to)

| Topic | Type | Frame | Planner use |
|---|---|---|---|
| `/Odometry_loc` | nav_msgs/Odometry | `camera_init` (odom) | odom→lidar edge in the transform tree |
| `/cloud_registered_1` | sensor_msgs/PointCloud2 | odom | occupancy / costmap input |

Notes: `/Odometry_loc` is the **sensor (head) pose** — it includes the head sway; the
registered cloud is sway-**compensated** (stable). For the robot base heading, the head
sway still has to be separated out (later work). The engine also broadcasts TF
`camera_init`→`body`.
