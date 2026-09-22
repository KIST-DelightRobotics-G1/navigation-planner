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
| Prior map | HF dataset | `maps/map.pcd` + `maps/map.uwb` — baked from [`Hanyu462/kist-g1-nav-map`](https://huggingface.co/datasets/Hanyu462/kist-g1-nav-map) (or record your own with `kist-map-recorder`) |

## Installation

#### 1. Clone Repository

```bash
git clone https://github.com/Safety-Node/kist-navigation-planner.git
cd kist-navigation-planner
```

All following steps run from the repository root.

#### Quick Start with Docker

The image bakes in everything — SDKs, toolchain, build, LIO engine, and the prior
map (pulled from the Hugging Face dataset at build time).

```bash
./docker/build.sh      # build the image
./docker/run.sh        # shell in the container; binaries under build/
```

This is the whole setup; the numbered steps 2–5 below are the manual
(non-Docker) alternative.

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

Localization needs a **prior map** of the environment. The Docker image already
bakes one from the [`Hanyu462/kist-g1-nav-map`](https://huggingface.co/datasets/Hanyu462/kist-g1-nav-map)
dataset. For a manual build, or to make a map of a new space, record one with the
included tool (drive/carry the robot around the space, Ctrl+C to save):

```bash
./build/kist-map-recorder            # writes maps/map.pcd (+ maps/map.uwb sidecar)
```

A new map is deployed by uploading the pair to the dataset and bumping the file
version in `docker/Dockerfile`.

## Build

With Docker, run this inside the container (`./docker/run.sh`).

```bash
cmake -B build && cmake --build build
```

## Usage

Set up the config once before running:

- `config/config.yaml` — DDS domain (`unitree.domain_id`) + transport
  (`config/cyclonedds.xml` holds the NIC), localization (`prior_map`,
  `map_uwb_yaw_deg`).
- `config/destinations.yaml` — named destination catalog + per-destination dock.
- All keys: [docs/configuration.md](docs/configuration.md).

Everything below runs inside the container (`./docker/run.sh`); the LIO engine and
a prior map are baked into the same image. Mapping a new space needs a new prior
map — see Installation step 5.

**`navigation.drive: true` MOVES THE ROBOT** — clear the area and keep gearsonic's e-stop in reach.

```bash
# 1. LIO engine — localization input (/Odometry_loc + /cloud_registered_1)
run_lio_daemon                            # start, detached (stop_lio_daemon to stop; tail -f /tmp/lio_engine.log)

# 2. navigation planner (preview by default; set navigation.drive: true in config.yaml to move)
./build/kist-navigation-planner
```

The planner publishes a `Twist` on `rt/kist/nav/cmd_vel`, but the robot only moves
once **kist-gearsonic-inference** is running **and its walk mode is manually
enabled** (see that repo) — nav being "up" is not enough on its own.

Command a destination and watch the result — the cortex orchestrator contract
(`SubtaskCmd` in on `rt/cortex/nav/cmd`; `SubtaskState` out on `rt/cortex/nav/state`
at 10 Hz: `IDLE / RUNNING / DONE / FAILED` + progress + note).

The test tools take an optional config path (default `config/config.yaml`).
Without the orchestrator, drive the contract by hand:

```bash
# command side — publish a subtask (rt/cortex/nav/cmd)
./build/test_subtask_cmd_send fridge      # move_to a named destination
./build/test_subtask_cmd_send --cancel    # cancel the current subtask

# state side — subscribe
./build/test_subtask_state_sub            # prints rt/cortex/nav/state (IDLE/RUNNING/DONE/FAILED)
./build/test_goal_recv                    # prints each command resolved to a Goal (debug)
```

An ad-hoc goal from rviz ("2D Goal Pose", odom frame) also works for quick tests.
