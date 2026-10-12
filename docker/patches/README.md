# LIO build patches

Patches applied to cloned dependencies during the Docker image build (see `docker/Dockerfile`).
Deps themselves are cloned at build time (not vendored host-side), so only our *changes* live in the
repo — as a small, reviewable diff.

## fast_lio-metrics-log.patch  ← ACTIVE (applied by the Dockerfile)
Applied to `deepglint/FAST_LIO_LOCALIZATION_HUMANOID`'s `FAST_LIO` at pinned commit
`df4772ec4797172430e7efe990711d09a529f4ad`.

**What it does:** LOG-ONLY instrumentation — pristine FAST-LIO plus one `printf` per scan in
`laserMapping.cpp`, right after the measurement update. It dumps the measurement-side signals we need
to design adaptive measurement covariance (soft down-weighting of the LiDAR): the pre→post pose
correction (`dpos`/`drot`), the matched-feature count (`feats`), the mean point-to-plane residual
(`res`), and the state velocity. **No rollback, no rejection — the estimator runs exactly as
upstream.** Grep the LIO log for `[lio_metrics]`.

Purpose: collect per-scan metrics during a manipulation (VLA) divergence so we can pick the right
signal to drive adaptive R. mat_pre.txt only logs the *state* (pos/vel/euler/bias/grav), not these
measurement-side quantities.

## fast_lio-bad-scan-reject.patch  ← ABANDONED (kept for reference, NOT applied)
The earlier hard scan-level reject + rollback. **Abandoned:** on-robot logs showed it *caused* the
divergence it was meant to prevent — `min_feats=50` (above the normal walking feature count ~44–49)
false-rejected good scans, skipped `map_incremental`, so the map stopped growing as the robot walked;
the robot outran the map → features collapsed → permanent rollback cut the LiDAR → pure-IMU runaway
(walk log: healthy, then 1270 m / 15 m/s). The pristine LIO (reject off) tracked the same walk fine
(2.8 m out-and-back, recovered). Hard output-override gates don't belong on the Kalman filter; the
replacement direction is adaptive R (soft, continuous, bounded). File left in place as history; the
Dockerfile no longer applies it.

**Applied in the Dockerfile with:** `patch -p1 -d /opt/lio_ws/src/fast_lio < ...`

### Re-generating / re-syncing
```bash
# clone upstream at the pinned commit, make your edits to a working copy, then:
diff -u a/src/laserMapping.cpp b/src/laserMapping.cpp > docker/patches/<name>.patch
# (a/ = pristine upstream, b/ = edited). Verify it applies:
patch -p1 --dry-run -d <pristine-FAST_LIO> < docker/patches/<name>.patch
```
If the pinned commit (`DEEPGLINT_FASTLIO_COMMIT` in the Dockerfile) changes, re-diff against the new
upstream so the patch still applies cleanly.
