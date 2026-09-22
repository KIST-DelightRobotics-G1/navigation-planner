# maps/ — prior LiDAR maps (machine-local, NOT in git)

The localization stack relocalizes the live LIO odom against a **prior PCD map** to produce
`map → odom` (see the frame tree in `include/frames/frame_ids.hpp`). That map is an
environment-specific, multi-MB artifact, so it is **gitignored** (`*.pcd`, `*.uwb`) and lives
here per machine, not in the repo history.

The Docker image bakes the deployed map from the public HF dataset
[`Hanyu462/kist-g1-nav-map`](https://huggingface.co/datasets/Hanyu462/kist-g1-nav-map)
(`docker/Dockerfile`). To deploy a new map: record it (below), upload the `.pcd` + `.uwb`
pair to the dataset, and bump the file version in the Dockerfile.

## Canonical prior map
```
maps/map.pcd     # the prior point cloud
maps/map.uwb     # UWB sidecar (deploy-time localization seed)
```
Point everything at this path (the relocalizer config + the destination survey). Frame = the
**odom frame of the mapping run that produced it** — this frame IS the `map` frame, and its origin
is the robot pose at **`run_lio_daemon` (LIO boot)**. Every destination in `config/destinations.yaml` must
be surveyed in THESE coordinates.

The `.uwb` sidecar has two lines:
```
UWB_x  UWB_y          # the UWB reading at capture
P_B_x  P_B_y  P_B_yaw # the robot's map pose at that instant
```
`P_B` records WHERE in the map the UWB was captured, so the deploy seed converts UWB → map by a
double transform (`map_xy = P_B + R(map_uwb_yaw)·(UWB_now − UWB_recorded)`) — correct even if the
recorder is started after the robot has moved from the LIO-boot origin.

## Producing / refreshing the prior map
FAST-LIO's built-in `pcd_save` is dead in our branch, so we record it ourselves with the
`kist-map-recorder` tool (accumulates `rt/cloud_registered_1` → voxel grid → PCD + UWB sidecar):
```
run_lio_daemon
./build/kist-map-recorder                     # HOLD STILL ~5 s first (origin UWB+P_B is averaged
#                                               over that window), THEN drive the whole environment
#   Ctrl+C to stop; if maps/map.* exist you are asked to confirm the overwrite
stop_lio_daemon
pcl_viewer maps/map.pcd                        # sanity-check: crisp walls, no ghost/double walls
```
The first ~5 s are averaged into the map-origin correspondence (`maps/map.uwb`), so keep the robot
stationary until you see `[MapRecorder] origin averaged over 5.0s (...)`. Then drive to map.
Output is always `maps/map.pcd` + `maps/map.uwb`. The voxel leaf is fixed at 0.05 m — to change it,
edit `MapRecorderConfig` in `include/recorder/map_recorder.hpp` and rebuild.

Re-record whenever the environment changes enough that relocalization degrades. Keep the map frame
stable (same `run_lio_daemon` origin) or destinations must be re-surveyed.
