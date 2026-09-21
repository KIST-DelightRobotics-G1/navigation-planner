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

Set up the config once before running:

- `config/config.yaml` — DDS domain (`unitree.domain_id`) + transport
  (`config/cyclonedds.xml` holds the NIC), and the localization block
  (`prior_map`, `map_uwb_yaw_deg`, relocalizer gates).
- `config/destinations.yaml` — the named destination catalog (map-frame `x/y/yaw`
  + per-destination dock: `align` / `approach` / `standoff_m`).

The planner is one of three cooperating processes — it needs the **LIO engine**
running (localization input) and **kist-gearsonic-inference** running to actually
move the robot: nav publishes a `Twist` on `rt/kist/nav/cmd_vel`, but gearsonic
only actuates it once its **walk mode is manually enabled**.

```bash
# preview only (no Twist published; robot will NOT move) — verify localization lock
./build/kist-navigation-planner

# drive (arms the Twist output; THE ROBOT WILL MOVE — clear the area, estop ready)
NAV_DRIVE=1 ./build/kist-navigation-planner
```

Command a destination by name and watch the result over DDS (the cortex
orchestrator contract):

- **in** — `SubtaskCmd` on `rt/cortex/nav/cmd`: `action: "move_to"`, `args: ["<destination>"]`
  (or `cancel: true`).
- **out** — `SubtaskState` on `rt/cortex/nav/state` at 10 Hz:
  `IDLE / RUNNING / DONE / FAILED` + progress + note.

Without a peer, drive the contract from the test tools:

```bash
./build/test_subtask_cmd_send fridge      # move_to fridge
./build/test_subtask_cmd_send --cancel    # cancel
./build/test_subtask_state_sub            # watch rt/cortex/nav/state
```

An ad-hoc goal from rviz ("2D Goal Pose", odom frame) also works for quick tests.

Tuning knobs (env, no rebuild): `NAV_VMAX` (cruise speed), `NAV_ARRIVAL_HOLD`
(arrival debounce, s), `NAV_NOPATH_HOLD` (no-path → FAILED debounce, s).
