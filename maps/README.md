# maps/ — prior LiDAR maps (machine-local, NOT in git)

The localization stack relocalizes the live LIO odom against a **prior PCD map** to produce
`map → odom` (see the frame tree in `include/frames/frame_ids.hpp`). That map is an
environment-specific, multi-MB artifact, so it is **gitignored** (`*.pcd`) and lives here per
machine, not in the repo history.

## Canonical prior map
```
maps/prior_map.pcd
```
Point everything at this path (the relocalizer config + the destination survey). Frame = the
**odom frame of the mapping run that produced it** — this frame IS the `map` frame, and every
destination in `config/destinations.yaml` must be surveyed in THESE coordinates.

## Producing / refreshing the prior map
FAST-LIO's built-in `pcd_save` is dead in our branch, so we record it ourselves with the
`map_recorder` tool (accumulates `rt/cloud_registered_1` → voxel grid → PCD):
```
lio_up
./build/map_recorder maps/prior_map.pcd      # drive the whole environment slowly
#   Ctrl+C to save
lio_down
pcl_viewer maps/prior_map.pcd                 # sanity-check: crisp walls, no ghost/double walls
```
Re-record whenever the environment changes enough that relocalization degrades. Keep the map
frame stable (same origin) or destinations must be re-surveyed.
