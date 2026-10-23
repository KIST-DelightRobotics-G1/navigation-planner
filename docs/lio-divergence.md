# FAST-LIO divergence — cause, attempts, and solution directions

**Status: in progress — 2026-10-21 raw-log analysis revised the root cause (feature starvation +
online-extrinsic drift, attitude-first ignition) and applied a config-only first fix on
`tune/lio-feature-density`; see the dated section at the end. Awaiting a verification run.**

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

## Measured case data

Raw logs live in `~/nav_diverge_logs/` on the dev machine. The two **diverged** cases' raw logs
(`matpre.txt` / `lio.log` / `trace`) are also committed in-repo under `docs/lio-divergence-logs/`
so other sessions/machines can read them directly. Distilled numbers below.

### Case index
| case | diverged? | summary |
|---|---|---|
| pristine-walk-443km | **YES** (pristine LIO) | ~14 min, mostly parked; velocity runaway w/ small dpos → 443,824 m |
| reject-walk-1270m | **YES** (our reject) | min_feats=50 false-rejects → map froze → IMU runaway → 1270 m |
| fullscenario-bounded | no | nav→VLA→handling, 8.7 min, \|pos\| ≤ 2.4 m, bounded |
| vla-trembling | no | manip-only (standing); feats 130→~60 + drot rise, self-recovered |
| noreject-clean | no | pristine walk, ~18 m out-and-back, recovered to 0.26 m |

### pristine-walk-443km — the real one

**A. Trajectory** (healthy ~700 s, then runaway; grav tilt stays nominal → not a gravity-state collapse):

| t(s) | \|pos\| (m) | \|vel\| (m/s) | grav_tilt (deg) |
|---|---|---|---|
| 0 | 0.02 | 0.02 | 1.6 |
| 122 | 0.64 | 0.09 | 1.5 |
| 244 | 1.36 | 0.12 | 1.3 |
| 365 | 1.42 | 0.14 | 1.1 |
| 487 | 1.40 | 0.14 | 1.2 |
| 608 | 1.41 | 0.18 | 1.2 |
| 730 | 2.48 | 0.69 | 1.2 |
| 851 | 12,607 | 354 | 1.1 |
| 973 | 107,623 | 1,217 | 1.2 |
| 1094 | 307,062 | 2,092 | 1.2 |

**B. Accel-bias \|ba\| vs \|vel\|** (80 s steps) — |ba| creeps 0.03→0.32 over ~12 min (poor stationary observability) while |vel| stays ~0.1, then both explode together:

| t(s) | \|ba\| | \|vel\| |
|---|---|---|
| 0 | 0.030 | 0.02 |
| 240 | 0.089 | 0.11 |
| 480 | 0.191 | 0.11 |
| 640 | 0.252 | 0.19 |
| 720 | 0.317 | 0.18 |
| 800 | 0.262 | 14.95 |
| 880 | 0.875 | 404.9 |
| 1040 | 3.048 | 1533.8 |
| 1200 | 3.415 | 2521.3 |

**C. Ignition** (`[lio_metrics]`, 10 s windows) — **velocity jumps 0.3→13→256 m/s while dpos (LiDAR correction) stays small and feats stays ~45 until later**; the LiDAR does NOT yank the pose, the velocity state runs away and the LiDAR can't arrest it (feats/res are lagging, not predictors):

| t(s) | feats_avg | dpos_max (m) | res_max (m) | vel_max (m/s) |
|---|---|---|---|---|
| 820–830 | 46 | 0.052 | 0.051 | 0.4 |
| 830–840 | 46 | 0.042 | 0.051 | 0.3 |
| 840–850 | 43 | 0.088 | 0.109 | 13.3 |
| 850–860 | 41 | 0.111 | 0.117 | 17.4 |
| 860–870 | 43 | 0.146 | 0.104 | 41.9 |
| 870–880 | 31 | 0.459 | 0.239 | 126.9 |
| 880–890 | 33 | 0.887 | 0.219 | 197.4 |
| 890–900 | 33 | 1.034 | 0.291 | 256.0 |

## 2026-10-21 — raw-log analysis and first config change

Analysed the committed raw logs (`docs/lio-divergence-logs/2026-10-13_pristine-walk_DIVERGED-443km`,
`matpre.txt` + `[lio_metrics]`) scan by scan. This revises the picture above.

### What the data shows
1. **Ten minutes of monotonic degradation before ignition, at (near) standstill.** 60 s means:

   | t (s) | feats | drot (°/scan) | res (m) | dpos (m) |
   |---|---|---|---|---|
   | 0–180 | 215 | 0.30 | 0.027 | 0.004 |
   | 300–360 | 127 | 0.49 | 0.032 | 0.007 |
   | 540–600 | 105 | 0.57 | 0.033 | 0.011 |
   | 660–720 | 62 | 0.75 | 0.039 | 0.018 |
   | 780–840 | 83 | 0.64 | 0.037 | 0.014 |
   | 840–900 | 42 | 1.62 | 0.067 | 0.036 |

   A static scene should give constant feats. Falling feats + rising residual at standstill = a
   positive feedback: jittery pose → blurred ikd-tree map → fewer plane fits → noisier pose.
2. **Ignition order is attitude → velocity, not velocity first.** 843–848 s: roll rotates
   −2° → +22° in 10 s (bg barely moves, so it is the LiDAR update doing it: drot 1–5°/scan with
   feats 17–45), and vel_y integrates to −8.5 m/s from the gravity leak (g·sin 20° ≈ 3.4 m/s²).
   dpos stays small because each misplaced scan is inserted into the map and the next scan matches
   it — the map follows the drift, so the LiDAR cannot arrest it. The earlier "velocity runs away
   while LiDAR agrees" reading was the same event seen from the wrong side.
3. **Three weakly-observable states were trading the same error.** With `extrinsic_est_en: true`
   the online IMU↔LiDAR extrinsic drifted to (−29, +40, −30) mm from the datasheet value (whose own
   magnitude is only (−11, −23, +44) mm) and 0.4–0.8° in rotation; `ba` grew only on the horizontal
   axes (x +0.25, y −0.26 m/s², non-physical for a MEMS bias); the gravity direction rotated 0.5°.
   Extrinsic and gravity have zero process noise in IKFoM, so once they absorb an error nothing
   pulls them back. The `|ba|` creep in the tables above is this, not a real bias.
4. **The nav trace covers only the 92 s walk, not the ignition**, so the suspected external
   lift/remote move at ~843 s could not be confirmed from the body IMU. Either way it was the last
   straw on top of (1)–(3), not the cause.
5. **Effective feature count ~45–80 is the sick number.** Upstream FAST-LIO typically runs with
   1,000+ effective points; the healthy first 3 min of this run had 215. 6-DoF attitude (roll in
   particular) needs floor/ceiling normals — at 45 points there was nothing holding roll.

### Change applied (branch `tune/lio-feature-density`) — `lio_engine/config/mid360.yaml`
| param | was | now | why |
|---|---|---|---|
| `point_filter_num` | 3 | 1 | keep every raw point |
| `filter_size_surf` | 0.5 | 0.2 | standard indoor Mid-360 value; hundreds-to-1000+ effective points |
| `filter_size_map` | 0.5 | 0.2 | paired with surf (upstream convention); denser map → better 5-NN plane fits |
| `extrinsic_est_en` | true | false | built-in IMU: extrinsic is a datasheet constant; online estimate drifted 4 cm |

Compute is not a constraint (LIO runs on a spare box, not the Jetson). Per-point information does
scale with N (overconfidence), but the pose is already pinned by the LiDAR at N=45, so raising N to
the upstream regime changes nothing qualitatively; bounding per-scan information (adaptive R) stays
an optional later step together with a robust kernel. The only real limit is plane quality at very
small voxels (< ~0.1 m), which 0.2 m stays clear of. Watch ikd-tree memory on long runs with the
0.2 m map voxel.

### How to verify — two runs, both required
Only the two *diverged* cases are committed in-repo; the VLA baselines (`vla_OK-trembling`,
`fullscenario_OK-bounded-2.4m`) live only in `~/nav_diverge_logs/` on the dev machine. From INDEX.md,
the old-param VLA baseline at the fridge was **feats 130 → ~60, drot 0.26 → 0.45°/scan** (bounded,
self-recovered). That is the "before" for run B; commit those logs too so the same per-minute table
can be produced for them.

**A. Near-standstill, 20 min** (reproduces the 443 km case). Pass = the table in (1) stays flat:
feats ≥ 150, drot ≈ 0.3°/scan, res ≈ 0.03 m; `ext_euler` / `offset_T_L_I` constant (they are now
fixed, so this is a sanity check); `|ba|` ≤ ~0.05 and not trending.

**B. Full nav → VLA (fridge) → handling → nav scenario** (the case this doc is actually for).
Pass, inside the manipulation window:
- feats stays ≥ 150 at the fridge (old: ~60). With ~1,000 points the arm/door points become a
  few-percent minority of the constraint instead of a 20 % one at feats 45, so the dynamic-point
  yank should *shrink*, not grow — this is the prediction to check.
- drot ≤ 0.5°/scan (old: 0.45 peak), no sustained one-signed roll/pitch drift in `euler`
  (the attitude-first signature from (2)).
- `|vel|` ≤ commanded + 0.3 m/s throughout; `|pos|` bounded by the fridge distance (old: 2.4 m).
- After the window: feats / res return to the pre-window level within ~10 s (no map pollution
  left behind by the door/arm).
If B fails while A passes, the remaining lever is the measurement side (robust kernel, item 3
below), not more tuning of these four params.

### Still open (next, in order)
1. Log-only patch: add the min eigenvalue of the 6×6 pos+rot Hessian (HᵀH) per scan — the proper
   degeneracy indicator (feats alone does not say which axis is unconstrained).
2. Small process noise on `grav` (and extrinsic if ever re-enabled) so they cannot freeze on a wrong value.
3. Robust kernel in `h_share_model` (#2 above) and, with it, per-scan information bounding.
