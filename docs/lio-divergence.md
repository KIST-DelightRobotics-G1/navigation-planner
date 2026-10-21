# FAST-LIO divergence — cause, attempts, and solution directions

**Status: deferred (hardest problem, lowest solve-probability). Pick up in a dedicated session.**

FAST-LIO odom occasionally blows up (pos → hundreds of thousands of m, vel → thousands m/s).
Goal: **keep LIO from diverging — especially during VLA (arm) manipulation — via a principled
estimator-level fix (not a ZUPT-style sidestep).** This doc is a self-contained handoff.

## System architecture (important)
- **Two-stage cascade (loosely-coupled):**
  - **Stage 1 — LIO ESKF** (IMU + LiDAR scan-matching; deepglint `FAST_LIO_LOCALIZATION_HUMANOID`,
    IKFoM, a separate ROS2 process) → odom pose.
  - **Stage 2 — nav localization EKF** (GICP + UWB) → map→odom, taking stage-1 odom as input.
- **UWB enters only Stage 2, not the LIO KF.** So UWB cannot arrest a Stage-1 divergence (Stage 2
  consumes the already-broken odom; the UWB-spike/LOST→relocalize path cannot recover it).
- FAST-LIO is cloned at docker-build time (pinned commit `df4772e`) and patched (`docker/patches/`).
  Active patch = **log-only instrumentation** (`fast_lio-metrics-log.patch`): one `printf` per scan,
  `[lio_metrics] dpos drot feats res vel` (behavior = pristine upstream). LIO console →
  `/tmp/lio_engine.log`. Per-scan state → `/opt/lio_ws/src/fast_lio/Log/mat_pre.txt`
  (columns: `t, euler(3), pos(3), ext_euler(3), offset_T_L_I(3), vel(3), bg(3), ba(3), grav(3)`;
  euler is unreliable near gimbal-lock / non-gravity-aligned frame → judge by pos/vel/grav).

## Root cause
- **Divergence trigger**: the Stage-1 ESKF goes unstable. Attitude error → gravity projected through
  the wrong attitude → fake horizontal accel (g·sinθ) → velocity integrates → position runaway
  (double integration → fast; self-sustaining while attitude stays wrong).
- **VLA-manipulation case (the fundamental one)**: FAST-LIO scan-matching assumes a **static world**.
  During manipulation the **arm + the moving fridge door** enter the near LiDAR FOV, injecting
  **dynamic points** into the scan. Scan-to-map matching mistakes them for "the map shifted" →
  applies a wrong correction → corrupts the ESKF → runaway.
- **Why it can't self-correct**: no absolute anchor inside the LIO loop → once it tips, nothing pulls
  it back.

## Divergence signature (measured; see `~/nav_diverge_logs/` archive + INDEX.md)
- Pristine divergence case (`pristine-walk-443km`): ~14 min in, robot mostly parked. **Velocity runs
  away 0.3 → 255 m/s while dpos (LiDAR correction) stays small (~0.09 m)**; feats/res only degrade
  afterward (lagging); grav magnitude preserved. Preceded by accel-bias |ba| creeping 0.03 → 0.32.
  → an ESKF velocity/attitude instability; **the earliest reliable signal is state |vel| exceeding
  the commanded speed** (feats/res/dpos are lagging, unusable as predictors).
- Pristine LIO handled plain walking and a full nav→VLA→handling scenario (8.7 min) without diverging
  (control cases in the archive). The problem is intermittent.

## Tried and abandoned
- **Hard bad-scan reject** (per-scan rollback if correction/feats/residual exceed thresholds):
  **abandoned — it caused the divergence it meant to prevent.** `min_feats=50` false-rejected normal
  walking scans (feats run 44–49) → skipped `map_incremental` → the map froze → feats→0 → permanent
  rollback cut the LiDAR → pure-IMU runaway (1270 m). Lesson: **hard-overriding the Kalman output
  (open-loop) blows up.** Patch kept for reference (`fast_lio-bad-scan-reject.patch`, not applied).

## Solution directions (priority / character)
1. **[principled, #1] Arm self-filtering (FK).** The robot knows its arm joint angles → forward
   kinematics → arm-link poses → geometrically drop LiDAR returns that hit its own arm (standard
   robot self-filter). Current `self_radius_m=0.35` only cuts a fixed radius; an extended arm reaches
   beyond it → need **FK-based dynamic masking**. Needs the arm joint-state feed + arm↔LiDAR
   extrinsic. Principled because those points are the known self, not the world to be estimated.
2. **[principled, #2] Robust estimation (IRLS / Huber / Cauchy)** in the scan-match cost → continuously
   down-weight points inconsistent with the static map (door / people) inside the optimization.
   FAST-LIO currently has only a binary point gate (`s>0.9`) in `h_share_model`; upgrade the
   per-point measurement weight to a robust kernel. Needs enough inliers → #1 (remove the arm) is a
   prerequisite. The abandoned hard-reject was a crude version of this.
3. **[auxiliary] Soft adaptive R** — scale `LASER_POINT_COV` up for suspect scans (stays closed-loop,
   bounded; unlike hard reject). Best driver = EWMA of the innovation (pose-correction) bias;
   per-scan magnitude / residual are blind to this failure.
4. **[sidestep, not preferred] ZUPT / velocity-sanity guard** — pin velocity=0 when stationary / reset
   on |vel| ≫ commanded. Effective but avoids LiDAR rather than fixing the estimator.
5. **[long-term] ULIO — tightly-couple UWB into the Stage-1 ESKF.** UWB as an absolute position anchor
   bounds position/velocity → prevents runaway + tracks the robot even during manipulation. Costs:
   large FAST-LIO surgery, UWB is position-only (no attitude), UWB ~15 cm noise + dropout/NLOS
   (and the repo has a pending "UWB no-fix fallback" reliability issue).
6. **[reinforcement] VLA-timing gate** — nav subscribes `rt/cortex/vla/state` to know the
   manipulation window and raise defenses (#1–#3 sensitivity) then. Doesn't catch non-manipulation
   divergence, so reinforcement only.

## Recommended direction
**#1 arm self-filter (FK) → #2 robust estimation (IRLS).** #1 removes the biggest disturbance (the
robot's own arm); #2 + the MID360 360° static majority (sides/rear) then carry the rest (door/people).
Both live in the FAST-LIO measurement stage (`h_share_model`, via `docker/patches/`).

## Assets
- Divergence log archive: `~/nav_diverge_logs/` (per-case `matpre/lio/trace` + `INDEX.md` analysis;
  `save_diverge.sh <container> <desc>` snapshots new cases). Not in the repo (large, gitignored).
- Instrumentation: `[lio_metrics]` in the LIO log + `mat_pre.txt`. nav trace via `tools/read_trace.py`.
