#!/usr/bin/env python3
"""Plot an acceptance-test run: scan-built occupancy map, true obstacles,
measured (simulator) trajectory, leg-odometry trajectory and the last plan.
usage: plot_run.py <log_dir> [out.png]"""
import sys, csv, pathlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

d = pathlib.Path(sys.argv[1]); out = sys.argv[2] if len(sys.argv) > 2 else str(d / "run.png")
rows = list(csv.DictReader(open(d / "trajectory.csv")))
def col(name, test=None):
    return np.array([float(r[name]) for r in rows if test is None or r["test"].startswith(test)])

fig, ax = plt.subplots(figsize=(11, 7.5))
mp = d / "map_final.csv"
if mp.exists():
    hdr = open(mp).readline().split()
    ox, oy, res = float(hdr[2]), float(hdr[4]), float(hdr[6])
    g = np.loadtxt(mp, delimiter=",", skiprows=1)
    img = np.full(g.shape + (3,), 1.0)
    img[g == 1] = (0.88, 0.95, 0.88)
    img[g == 2] = (0.1, 0.1, 0.1)
    ax.imshow(img, origin="lower", extent=(ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res), zorder=0)
for (cx, cy, hx, hy, c) in [(2.5, 0, .2, .6, "tab:orange"), (3.6, -1.0, .3, .3, "gold")]:
    ax.add_patch(Rectangle((cx - hx, cy - hy), 2 * hx, 2 * hy, fill=False, ec=c, lw=2.5, zorder=3))
ax.add_patch(Circle((4.0, 1.1), 0.2, fill=False, ec="tab:orange", lw=2.5, zorder=3))
colors = {"2": "tab:gray", "3": "tab:gray", "4_": "tab:blue", "4b": "tab:cyan", "6": "tab:red", "8": "tab:purple"}
labels = {"2": "2-3 open-loop walk/turn", "4_": "4 goTo(0,2.5) free space", "4b": "return to A (0,0, yaw 0)",
          "6": "6-7 goTo(5,0) around obstacles", "8": "8 unreachable goal (fails safely)"}
for k, c in colors.items():
    x, y = col("gt_x", k), col("gt_y", k)
    if len(x): ax.plot(x, y, color=c, lw=2.2, label=labels.get(k), zorder=4)
ox_, oy_ = col("odo_x", "6"), col("odo_y", "6")
if len(ox_): ax.plot(ox_, oy_, "--", color="tab:red", lw=1, alpha=0.6, label="leg odometry estimate (test 6)", zorder=4)
pp = d / "last_path.csv"
if pp.exists():
    p = np.loadtxt(pp, delimiter=",", skiprows=1, ndmin=2)
    if len(p): ax.plot(p[:, 0], p[:, 1], "o:", color="k", ms=3, lw=1, label="last planned path", zorder=5)
ax.plot(0, 0, "s", color="green", ms=10, zorder=6); ax.annotate("A", (0.05, 0.12), fontsize=13, weight="bold")
ax.plot(5, 0, "*", color="green", ms=16, zorder=6); ax.annotate("B", (5.05, 0.12), fontsize=13, weight="bold")
ax.set_xlim(-1.5, 6.5); ax.set_ylim(-2.5, 3.5); ax.set_aspect("equal"); ax.grid(alpha=0.3)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("Go2 in unitree_mujoco: measured trajectory (sportmodestate) over scan-built occupancy map\n"
             "black = cells marked occupied by the robot's own scanner, outlines = true obstacles")
ax.legend(loc="upper right", fontsize=8)
fig.tight_layout(); fig.savefig(out, dpi=110)
print("wrote", out)
