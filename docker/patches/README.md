# LIO build patches

Patches applied to cloned dependencies during the Docker image build (see `docker/Dockerfile`).
Deps themselves are cloned at build time (not vendored host-side), so only our *changes* live in the
repo — as a small, reviewable diff.

## fast_lio-bad-scan-reject.patch
Applied to `deepglint/FAST_LIO_LOCALIZATION_HUMANOID`'s `FAST_LIO` at pinned commit
`df4772ec4797172430e7efe990711d09a529f4ad`.

**What it does:** rejects a physically-impossible LiDAR correction in `laserMapping.cpp` — snapshots
the IMU-predicted state before the measurement update, and if the pre→post correction is too large
(position/rotation), or the match is degenerate (too few effective features / high residual), rolls
it back (`change_x`/`change_P`) and skips the map insertion for that scan. This stops a transient bad
scan (a fridge door / arm sweeping the near LiDAR FOV during manipulation) from diverging the odom.
Tunable via `reject.*` in `lio_engine/config/mid360.yaml`.

**Applied in the Dockerfile with:** `patch -p1 -d /opt/lio_ws/src/fast_lio < ...`

### Re-generating / re-syncing
```bash
# clone upstream at the pinned commit, make your edits to a working copy, then:
diff -u a/src/laserMapping.cpp b/src/laserMapping.cpp > docker/patches/fast_lio-bad-scan-reject.patch
# (a/ = pristine upstream, b/ = edited). Verify it applies:
patch -p1 --dry-run -d <pristine-FAST_LIO> < docker/patches/fast_lio-bad-scan-reject.patch
```
If the pinned commit (`DEEPGLINT_FASTLIO_COMMIT` in the Dockerfile) changes, re-diff against the new
upstream so the patch still applies cleanly.
