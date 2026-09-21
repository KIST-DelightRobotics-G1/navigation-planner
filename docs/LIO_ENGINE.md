# LIO engine (DDS service, same image as the planner)

LIO runs as a **separate ROS2 process**, not linked into this ROS-free planner — but it
lives in the **same all-in-one image** (`docker/Dockerfile`) as the planner. It is the
upstream **deepglint FAST_LIO_LOCALIZATION_HUMANOID** stack (raw Livox driver + FAST-LIO),
built unmodified into `/opt/lio_ws`. Engine and planner run as two processes in one
container and talk **only over DDS** on localhost (RTPS interop, FastDDS↔CycloneDDS,
validated), so ROS2 never enters the planner's build or process.

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

Both commits are pinned as ARGs in **`docker/Dockerfile`**, which builds one all-in-one
image (planner + engine): it clones deepglint at build time, copies each package from its
pinned commit into `/opt/lio_ws`, and colcon-builds them alongside the planner. Nothing is
vendored into the repo (the clone is ~640 MB, build-time only).

## Build

```bash
./docker/build.sh          # planner + LIO engine, one image (tag: kist-navigation-planner)
```

Our own config lives in **`lio_engine/config/`** (version-controlled here) and is used by
`lio_engine/lio_bringup.launch.py`, not the upstream defaults:
`MID360_config.json` → `extrinsic_parameter.roll = 180` (upside-down mount), lidar ip
`192.168.123.120`, host_net_info ip `192.168.123.222` (must match the host's robot-LAN IP);
`mid360.yaml` → `lid_topic: /livox/lidar`, `dense_publish_en: false` (registered cloud is a
downsampled scan; set true for the full ~20k points).

## Run

Two commands on the **host**, everything else **inside the container** (`env.sh` is
auto-sourced, so `ros2` + `run_lio_daemon` / `stop_lio_daemon` are ready in every shell):

```bash
# host
./docker/build.sh          # build the all-in-one image (once, or after Dockerfile edits)
./docker/run.sh            # enter the container (--network host, X11, source mounted)

# inside the container (binaries are already baked/built — no cmake needed)
run_lio_daemon                                          # start the engine (driver + FAST-LIO), detached
./build/test_lio_receiver config/config.yaml    # planner consumes the engine over DDS
stop_lio_daemon                                         # stop the engine
tail -f /tmp/lio_engine.log                      # watch engine logs
```

The image is self-contained (source + binaries baked by `docker build`), so `run.sh`
drops you into a container with ready binaries under `build/` — no in-container build.
For iterative dev, uncomment the source bind-mount in `docker/run.sh` and rebuild inside.

`run_lio_daemon` runs `ros2 launch .../lio_engine/lio_bringup.launch.py` — driver + FAST-LIO
together, using **our** config (`lio_engine/config/`), no manual multi-terminal. To change
topics / config / what launches, edit `lio_engine/`; upstream source is untouched.

Prereq: host is on the robot LAN (`192.168.123.222`), lidar reachable
(`ping 192.168.123.120`). `run_lio_daemon` warns if it is not.

Optional rviz (inside the container; `run.sh` forwards X): `rviz2` — add displays by hand:
Fixed Frame `camera_init`, PointCloud2 on `/cloud_registered_1` (raise Decay Time), Odometry
on `/Odometry_loc`.

## Topic contract (what the planner subscribes to)

| Topic | Type | Frame | Planner use |
|---|---|---|---|
| `/Odometry_loc` | nav_msgs/Odometry | `camera_init` (odom) | odom→lidar edge in the transform tree |
| `/cloud_registered_1` | sensor_msgs/PointCloud2 | odom | occupancy / costmap input |

Notes: `/Odometry_loc` is the **sensor (head) pose** — it includes the head sway; the
registered cloud is sway-**compensated** (stable). For the robot base heading, the head
sway still has to be separated out (later work). The engine also broadcasts TF
`camera_init`→`body`.
