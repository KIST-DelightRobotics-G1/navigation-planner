#!/usr/bin/env python3
"""Read logs/latest.trace (NavTrace) into numpy.

    from read_trace import load
    t = load("logs/latest.trace")
    t["cmd_vx"]           # one column as a 1-D array
    t.cols("imu_quat")    # all imu_quat_* columns as an (N, 4) array
    t.time                # seconds since the trace started

CLI:
    python3 tools/read_trace.py logs/latest.trace                       # summary
    python3 tools/read_trace.py logs/latest.trace --last                # last recorded pose/state (+ map position)
    python3 tools/read_trace.py logs/latest.trace --plot cmd_vx goal_dist progress
    python3 tools/read_trace.py logs/latest.trace --csv out.csv --cols follow_phase cmd_vx --from 120 --to 135
        # --cols: column names, or a group prefix (imu_quat, imu_gyro, ...) for all of it;
        #         omit to export every column. --from/--to: seconds since trace start.
"""
import argparse
import re
import sys

import numpy as np

# enum decodings (match the C++ headers) — for the summary only.
FOLLOW_PHASE = ["Driving", "Aligning", "Approaching", "Arrived", "Blocked", "NoPath", "Idle"]
SUBTASK_STATUS = ["Idle", "Running", "Done", "Failed"]
LOC_PHASE = ["search", "track", "lock-init", "lock-fixed", "uwb-spike", "lost"]


class Trace:
    def __init__(self, names, data):
        self.names = names
        self.index = {n: i for i, n in enumerate(names)}
        self.data = data

    def __getitem__(self, name):
        return self.data[:, self.index[name]]

    def cols(self, prefix):
        """All columns named <prefix>_<k>, ordered by k, as (N, K)."""
        pat = re.compile(rf"^{re.escape(prefix)}_(\d+)$")
        ks = sorted((int(m.group(1)), n) for n in self.names if (m := pat.match(n)))
        return self.data[:, [self.index[n] for _, n in ks]]

    @property
    def time(self):
        return self["t_s"]

    def __len__(self):
        return self.data.shape[0]

    def select(self, cols=None, t_from=None, t_to=None):
        """(names, rows) for the given columns (names or group prefixes) and
        time window in seconds. Always includes t_s first."""
        names = []
        for c in cols or self.names:
            if c in self.index:
                names.append(c)
            else:
                grp = [n for n in self.names if re.match(rf"^{re.escape(c)}_\d+$", n)]
                if not grp:
                    raise KeyError(f"unknown column or group: {c}")
                names.extend(sorted(grp, key=lambda n: int(n.rsplit("_", 1)[1])))
        if "t_s" in names:
            names.remove("t_s")
        names.insert(0, "t_s")
        mask = np.ones(len(self), dtype=bool)
        if t_from is not None:
            mask &= self.time >= t_from
        if t_to is not None:
            mask &= self.time <= t_to
        return names, self.data[mask][:, [self.index[n] for n in names]]

    def to_csv(self, path, cols=None, t_from=None, t_to=None):
        names, rows = self.select(cols, t_from, t_to)
        np.savetxt(path, rows, delimiter=",", fmt="%.6g", header=",".join(names), comments="")
        return len(rows), len(names)


def load(path):
    with open(path, "rb") as f:
        head = b""
        while not head.endswith(b"END\n"):
            chunk = f.readline()
            if not chunk:
                raise ValueError("no END marker in header")
            head += chunk
        lines = head.decode().splitlines()
        if lines[0] != "NAVTRACE v1":
            raise ValueError(f"unexpected magic: {lines[0]}")
        ncol = int(lines[1].split("=")[1])
        names = lines[3].split(",")
        if len(names) != ncol:
            raise ValueError(f"header says {ncol} columns, lists {len(names)}")
        raw = np.frombuffer(f.read(), dtype="<f4")
    rows = raw.size // ncol
    return Trace(names, raw[: rows * ncol].reshape(rows, ncol))


def _decode(labels, v):
    i = int(round(v))
    return labels[i] if 0 <= i < len(labels) else str(i)


def print_last(t):
    """The last recorded tick: where the robot was and what it was doing."""
    import math
    if not len(t):
        print("(empty trace)")
        return
    r = -1
    def g(name):
        return t[name][r] if name in t.index else float("nan")
    ts = t.time[r]
    print(f"last tick @ t={ts:.2f}s  (tick {int(g('tick'))}, dropped {int(g('trace_dropped'))})")
    print(f"  controller : {_decode(FOLLOW_PHASE, g('follow_phase'))} / "
          f"{_decode(SUBTASK_STATUS, g('subtask_status'))}  "
          f"prog={g('progress'):.2f}  cmd=({g('cmd_vx'):+.2f},{g('cmd_vy'):+.2f},{g('cmd_vyaw'):+.2f})  "
          f"{'DRIVE' if g('drive_enabled') > 0.5 else 'preview'}")
    print(f"  odom pelvis: x={g('opel_x'):.2f} y={g('opel_y'):.2f} z={g('opel_z'):.2f}  yaw={math.degrees(g('opel_yaw')):.1f} deg")
    if g("lev_valid") > 0.5:
        print(f"  leveled    : x={g('lpel_x'):.2f} y={g('lpel_y'):.2f}  yaw={math.degrees(g('lpel_yaw')):.1f} deg  "
              f"floor tilt={g('lev_tilt_deg'):.2f} deg")
    if g("loc_valid") > 0.5:
        mx, my, myaw = g("mapodom_x"), g("mapodom_y"), g("mapodom_yaw")
        # robot position in MAP = T_map_odom ∘ odom_pelvis
        c, s = math.cos(myaw), math.sin(myaw)
        rmx = mx + c * g("opel_x") - s * g("opel_y")
        rmy = my + s * g("opel_x") + c * g("opel_y")
        print(f"  localization: {_decode(LOC_PHASE, g('loc_phase'))}  "
              f"T_map_odom=({mx:.2f},{my:.2f},{math.degrees(myaw):.1f} deg)  gicp_fit={g('gicp_fitness'):.3f}")
        print(f"  robot in MAP: x={rmx:.2f} y={rmy:.2f}   (uwb map: {g('uwb_map_x'):.2f},{g('uwb_map_y'):.2f}, have_uwb={int(g('have_uwb'))})")
    if g("goal_valid") > 0.5:
        print(f"  goal       : x={g('goal_x'):.2f} y={g('goal_y'):.2f}  dist={g('goal_dist'):.2f} m  "
              f"(map goal={int(g('goal_in_map'))}, idx={int(g('subtask_index'))})")


def _spans(codes, labels):
    """Contiguous [(label, count)] runs of an integer code column."""
    out = []
    for c in codes.astype(int):
        name = labels[c] if 0 <= c < len(labels) else str(c)
        if out and out[-1][0] == name:
            out[-1][1] += 1
        else:
            out.append([name, 1])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="logs/latest.trace")
    ap.add_argument("--last", action="store_true", help="print the last recorded tick (pose, state, map position)")
    ap.add_argument("--plot", nargs="*", help="column names to plot against time")
    ap.add_argument("--csv", help="write the selected columns/window as CSV to this file")
    ap.add_argument("--cols", nargs="*", help="columns or group prefixes for --csv (default: all)")
    ap.add_argument("--from", dest="t_from", type=float, help="window start, seconds since trace start")
    ap.add_argument("--to", dest="t_to", type=float, help="window end, seconds since trace start")
    a = ap.parse_args()
    t = load(a.path)
    if a.csv:
        n, k = t.to_csv(a.csv, a.cols, a.t_from, a.t_to)
        print(f"wrote {a.csv}: {n} rows x {k} columns")
        return
    if a.last:
        print_last(t)
        return
    dur = t.time[-1] - t.time[0] if len(t) else 0.0
    print(f"{a.path}: {len(t)} ticks, {dur:.1f} s, {len(t.names)} columns, "
          f"dropped {int(t['trace_dropped'][-1]) if len(t) else 0}")
    if not len(t):
        return
    print(f"tick compute us (max/mean): {t['tick_us'].max():.0f} / {t['tick_us'].mean():.0f}")
    # localization phases seen, and lock losses
    phases = _spans(t["loc_phase"], LOC_PHASE)
    seen = ", ".join(sorted({p for p, _ in phases}))
    n_lost = sum(1 for p, _ in phases if p == "lost")
    print(f"loc phases: {seen}   (lost->reinit x{n_lost})")
    # driving segments (follow_phase != Idle while a goal is active)
    active = t["goal_valid"] > 0.5
    if active.any():
        run_s = t.time[active][-1] - t.time[active][0]
        print(f"goal active for {int(active.sum())} ticks (~{run_s:.1f} s window); "
              f"max goal_dist {t['goal_dist'][active].max():.2f} m, final progress {t['progress'][active][-1]:.2f}")
    if a.plot:
        import matplotlib.pyplot as plt
        for name in a.plot:
            plt.plot(t.time, t[name], label=name)
        plt.xlabel("s"); plt.legend(); plt.grid(True); plt.show()


if __name__ == "__main__":
    sys.exit(main())
