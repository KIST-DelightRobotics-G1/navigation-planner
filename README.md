# kist-navigation-planner

C++ autonomous navigation planner for the Unitree G1 humanoid robot (ROS-free, DDS).

## Architecture

[![Architecture](docs/kist-navigation-planner.svg)](docs/kist-navigation-planner.svg)

## Dependencies

| Component | Version | Role |
|---|---|---|
| `unitree_sdk2` | `21d0a3b` | Unitree G1 DDS client (LIO / UWB / lowstate in, cmd_vel out) |
| CycloneDDS + CycloneDDS-CXX | 0.10.2 | `idlc`/`idlcxx` codegen for the nav DDS types |
| PCL | distro | prior-map GICP relocalization |
| Eigen3 | distro | linear algebra (transforms, EKF) |
| `yaml-cpp` | distro | config parsing |
| FAST-LIO localization engine | deepglint (external) | LiDAR odometry + registered cloud over DDS ([docs/LIO_ENGINE.md](docs/LIO_ENGINE.md)) |
| Prior map | recorded | `maps/map.pcd` + `maps/map.uwb` (from `kist-map-recorder`) |

## Installation

#### 1. Clone Repository

```bash
git clone https://github.com/Safety-Node/kist-navigation-planner.git
cd kist-navigation-planner
```

All following steps run from the repository root.

#### Quick Start with Docker

The image bakes in everything — SDKs, toolchain, build. `build.sh` picks the
Dockerfile for your architecture (x86 / Jetson).

```bash
./docker/build.sh      # build the image
./docker/run.sh        # shell in the container; binaries under build/
```

This is the whole setup; the numbered steps 2–5 below are the manual
(non-Docker) alternative. The prior map is a runtime artifact you provide — see
step 5.

#### 2. Install unitree_sdk2

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2
git -C thirdparty/unitree_sdk2 checkout 21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
```

#### 3. Install apt packages

```bash
sudo apt update && sudo apt install -y \
    build-essential cmake git pkg-config \
    libyaml-cpp-dev libeigen3-dev libpcl-dev
```

#### 4. Install CycloneDDS (idlc toolchain)

CycloneDDS + CycloneDDS-CXX 0.10.2 into `/opt/cyclonedds`, pinned to match the
SDK's bundled `libddscxx` (required — the nav DDS types are generated from
`idl/kist_nav.idl` at build time):

```bash
git clone --depth 1 -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/cyclonedds
cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build \
    -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -DBUILD_IDLC=ON -DCMAKE_BUILD_TYPE=Release
sudo cmake --build /tmp/cyclonedds/build --target install -j"$(nproc)"

git clone --depth 1 -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git /tmp/cyclonedds-cxx
cmake -S /tmp/cyclonedds-cxx -B /tmp/cyclonedds-cxx/build \
    -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -DCMAKE_PREFIX_PATH=/opt/cyclonedds -DCMAKE_BUILD_TYPE=Release
sudo cmake --build /tmp/cyclonedds-cxx/build --target install -j"$(nproc)"

export PATH=/opt/cyclonedds/bin:$PATH      # idlc on PATH for Build
```

#### 5. LIO engine + prior map

The planner does not run SLAM itself — it consumes an **external FAST-LIO
localization service** over DDS (odometry on `/Odometry_loc`, registered cloud on
`/cloud_registered_1`). Set it up per [docs/LIO_ENGINE.md](docs/LIO_ENGINE.md).

Localization needs a **prior map** of the environment. Record one with the
included tool (drive/carry the robot around the space, Ctrl+C to save), or drop an
existing pair in place:

```bash
./build/kist-map-recorder            # writes maps/map.pcd (+ maps/map.uwb sidecar)
```

## Build

With Docker, run this inside the container (`./docker/run.sh`).

```bash
cmake -B build && cmake --build build
```

## Usage

Set up the config once before running (all keys:
[docs/configuration.md](docs/configuration.md)):

- `config/config.yaml` — DDS domain + transport (the NIC lives in
  `config/cyclonedds.xml`) and the localization block (`prior_map`,
  `map_uwb_yaw_deg`, relocalizer gates).
- `config/destinations.yaml` — the named destination catalog + per-destination dock.

Everything below runs inside the container (`./docker/run.sh`); the LIO engine is
baked into the same image. A new space needs a prior map first — record one with
`./build/kist-map-recorder` (see Installation step 5).

**`NAV_DRIVE=1` MOVES THE ROBOT** — clear the area and keep gearsonic's e-stop in reach.

```bash
# 1. LIO engine — localization input (/Odometry_loc + /cloud_registered_1)
lio_up                                    # start, detached (lio_down to stop; tail -f /tmp/lio_engine.log)

# 2. navigation planner
./build/kist-navigation-planner           # preview: no Twist, robot will NOT move (verify the lock)
NAV_DRIVE=1 ./build/kist-navigation-planner   # drive: arms the Twist output — THE ROBOT WILL MOVE
```

The planner publishes a `Twist` on `rt/kist/nav/cmd_vel`, but the robot only moves
once **kist-gearsonic-inference** is running **and its walk mode is manually
enabled** (see that repo) — nav being "up" is not enough on its own.

Command a destination and watch the result — the cortex orchestrator contract
(`SubtaskCmd` in on `rt/cortex/nav/cmd`; `SubtaskState` out on `rt/cortex/nav/state`
at 10 Hz: `IDLE / RUNNING / DONE / FAILED` + progress + note). Without the
orchestrator, drive it from the test tools:

```bash
./build/test_subtask_state_sub            # watch rt/cortex/nav/state
./build/test_subtask_cmd_send fridge      # move_to a named destination
./build/test_subtask_cmd_send --cancel    # cancel the current subtask
```

An ad-hoc goal from rviz ("2D Goal Pose", odom frame) also works for quick tests.
