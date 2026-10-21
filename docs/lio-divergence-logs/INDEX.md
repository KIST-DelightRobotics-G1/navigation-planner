# LIO divergence case archive

Logs collected while chasing the FAST-LIO divergence problem (the hardest, deferred-to-last item).
Each case folder holds `matpre.txt` (per-scan LIO state: t, euler, pos, ext, offT, vel, bg, ba, grav),
`lio.log` (LIO console — includes `[lio_metrics]` per-scan dpos/drot/feats/res/vel once the metrics
patch is in the image; append-log, may span runs), and `trace` (nav per-tick, read with
`tools/read_trace.py`). Add new cases with `./save_diverge.sh <container> <desc>`.

**Column key (matpre.txt):** `[0]t  [1:4]euler(rpy)  [4:7]pos  [7:10]ext_euler  [10:13]offset_T_L_I  [13:16]vel  [16:19]bg  [19:22]ba  [22:25]grav`
Note: euler is unreliable near gimbal-lock / non-gravity-aligned frame — judge by pos/vel/grav.

## Cases

### 2026-10-13_pristine-walk_DIVERGED-443km  ★ the real one
- Pristine LIO (reject OFF, metrics patch). ~20 min run, mostly parked (~1.4 m) after a short walk.
- **DIVERGED at t≈843 s** (14 min): velocity state ran away 0.3→255 m/s, |pos|→443,824 m.
- Signature: **vel explodes while dpos (LiDAR correction) stays small (~0.09 m)**; feats ~45 until ~870 s (collapse is a consequence); grav magnitude preserved (−9.81). → ESKF velocity/attitude runaway the LiDAR fails to arrest; NOT the reject failure, NOT a wrong-LiDAR yank.
- Preceding: |ba| (accel bias) crept 0.03→0.32 over ~12 min (poor stationary observability?). Suspected trigger: **external controller manipulation** (lift/jerky remote move) — unconfirmed.
- Next step to settle cause: round-trip test WITHOUT external manipulation (does it still diverge?).

### 2026-10-12_reject-walk_DIVERGED-1270m
- Our hard bad-scan reject (min_feats=50) caused this. Walking feats run 44–49 (< 50) → false-rejected good scans → skipped map_incremental → map didn't grow → feats→0 → permanent rollback cut the LiDAR → pure-IMU runaway (healthy ~130 s then →1270 m / 15 m/s). Reason the reject was abandoned.

### 2026-10-12_fullscenario_OK-bounded-2.4m  (baseline, no divergence)
- nav→VLA→human-move→nav→VLA, 8.7 min, pristine. |pos| max 2.39 m (= fridge distance), bounded, grav nominal. Did NOT diverge — shows pristine LIO handles the full realistic scenario.

### 2026-10-12_vla_OK-trembling  (baseline, no divergence)
- Manipulation-only (standing). No divergence, but trembling precursor: at the fridge feats drop 130→~60 (occlusion/low-feature), drot rises 0.26→0.45° — bounded, self-recovered.

### 2026-10-12_noreject_OK-clean-recovered  (baseline, no divergence)
- Pristine walk (reject off). Robot walked ~18 m out-and-back, tracked cleanly, returned to 0.26 m. The control proving pristine LIO handles walking; the earlier "walk divergence" was our reject.

## Summary of the divergence problem (as of 2026-10-16)
- Pristine LIO is robust to walking + a full nav+VLA scenario (baselines above).
- The one pristine divergence (443km case) came after long operation / suspected external manipulation; velocity runs away with the LiDAR correction staying small → intrinsic ESKF instability, not fixable by the UWB-spike/relocalize path (that re-aligns map→odom, not the broken LIO odom).
- Deferred to last (lowest solve-probability). When resumed: first confirm trigger (external-manip vs intrinsic) via a no-manipulation round-trip; then consider ZUPT (stationary) / velocity-sanity guard / detect-manipulation-and-reinit.
