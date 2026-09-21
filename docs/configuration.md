# Configuration

All runtime configuration for kist-navigation-planner. Files are read at startup
(restart to reload); env overrides are read at launch.

## `config/config.yaml`

Holds only what the running system reads — the DDS transport and the localization
block. Route-planner tuning (obstacle grid, costmap, A*, smoother) lives as
calibrated defaults in the `route_planner/` headers.

### `unitree`

| Key | Default | Meaning |
|---|---|---|
| `domain_id` | `0` | DDS domain id. Change to isolate multiple robots on one network. |
| `dds_config` | `config/cyclonedds.xml` | Path to the CycloneDDS transport XML. The **network interface** and socket-buffer / defrag tuning live in the XML, not here. |

### `localization`

Relocalizes the live LIO odom against a prior PCD map → `map→odom`, so map-frame
destinations land on the same physical spot every run. A missing `prior_map`
disables localization (rviz odom goals still work).

| Key | Default | Meaning |
|---|---|---|
| `prior_map` | `maps/map.pcd` | Prior point-cloud map (written by `kist-map-recorder`, with the `maps/map.uwb` sidecar). |
| `seed` | `{x: 0, y: 0, yaw_deg: 0}` | Fixed fallback `map←odom` guess used when UWB / sidecar is unavailable. `yaw_deg` doubles as the UWB-seed heading — match your boot heading. |
| `map_uwb_yaw_deg` | `90.0` | `map←UWB` rotation applied to the UWB displacement. The UWB anchor axes are ~90° rotated from odom (UWB x=left/y=back vs odom x=fwd/y=left). Fine-tune 88–92 if the seed lands off. |
| `fitness_max` | `0.05` | LOCK only if GICP fitness (mean sq err, m²) ≤ this. Tighten for meaningful locks (0.30 accepts ~0.4 m locks; 0.05 ≈ RMS 0.22 m). |
| `max_jump_m` | `1.0` | Reject a correction translating `map→odom` more than this (m). |
| `max_yaw_deg` | `30` | Reject a solution more than this many degrees from the seed heading (kills 180° flips). |

### `navigation`

Runtime toggles (restart to apply).

| Key | Default | Meaning |
|---|---|---|
| `drive` | `false` | `false` = preview (no Twist; robot will NOT move). `true` = arm the Twist output on `rt/kist/nav/cmd_vel` → gearsonic. **The robot moves.** |
| `survey` | `false` | `true` = print the robot's MAP-frame pose each cycle, to survey a spot's `x/y/yaw` for `destinations.yaml`. |

## `config/cyclonedds.xml`

The DDS transport. Edit this to select the **network interface** (the robot LAN
NIC) and to tune socket buffers / fragmentation. The SDK routes everything through
this XML (see `common/dds_config.hpp`).

## `config/destinations.yaml`

The named-destination catalog. The robot is commanded to one by name over
`rt/cortex/nav/cmd` (`SubtaskCmd`, `action: move_to`, `args: ["<name>"]`); the
planner looks the name up here, plans + drives to `(x, y)`, then runs the
per-destination dock. Edited without a rebuild (loaded at startup).

Each entry:

| Key | Meaning |
|---|---|
| `name` | Destination name (what the command carries). |
| `x`, `y` | Map-frame goal, metres (+x forward, +y left). |
| `yaw_deg` | Arrival heading, degrees CCW from +x (the pose the robot faces after reaching `x, y`). |

Optional per-destination **dock** (default off → just stop at the arrival tolerance):

| Key | Default | Meaning |
|---|---|---|
| `align` | `false` | Rotate in place to `yaw_deg` on arrival. |
| `approach` | `false` | After aligning, creep straight forward until the object ahead is `standoff_m` away. |
| `standoff_m` | `0.40` | Approach stop distance to the nearest lethal cell ahead. |
| `trigger_m` | `1.20` | Approach engages only if an object is within this distance ahead. |
| `speed` | `0.35` | Approach creep speed (m/s; loco floor ≈ 0.2). |

> Tip: keep the goal `(x, y)` clear of the object's lethal disk (~0.5 m) and let
> `approach` hold the final standoff — a goal buried in lethal makes A* route
> through it and can trip the reactive stop before docking engages.

## Environment overrides

Read at launch (no rebuild):

| Variable | Applies to | Meaning |
|---|---|---|
| `NAV_VMAX` | `kist-navigation-planner` | Follower cruise speed (m/s). |
| `NAV_ARRIVAL_HOLD` | `kist-navigation-planner` | Arrival debounce (s): hold at the goal this long before reporting `DONE` (default 1.0). |
| `NAV_NOPATH_HOLD` | `kist-navigation-planner` | No-path debounce (s): a transient no-path stays `RUNNING`; only a sustained one this long → `FAILED` "no path" (default 4.0). |

Route-planner tuning (obstacle grid / costmap / A* / smoother) can be swept
without a rebuild via the env overrides in `test_obstacle_grid` (`MIN_H`,
`LETHAL_R`, `W_CENTER`, `R_MAX`, …); the accepted defaults live in the
`route_planner/` headers.
