#!/usr/bin/env python3
"""Live plot of the simulator's streamed pose and IR readings on a 16x16 maze.

Usage:
    .pio/build/sim/program | python3 tools/simviz.py              # live window
    python3 tools/simviz.py sim.log --save run.png                # static image of a saved run

Reads the maze printed once at startup, then the blocks printed every tick:
    MAZE: <w> <h> <w*h 0/1 chars, row by row from y = 0>   (SimIO::printMaze)
    TIME: <seconds since start>
    WORLD: <x>, <y> THETA: <theta>
    0: <x>, <y>     1: <x>, <y>     2: ...      (IR hits, robot frame, metres)
"""
import argparse
import math
import sys
import threading

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon, Rectangle

N = 16
CELL_SIZE_METERS = 0.18  # lib/core/Constants.h
CENTER_GOAL_CELLS = [(7, 7), (7, 8), (8, 7), (8, 8)]
MIN_STEP_METERS = 0.002  # skip path points closer than this to keep the trail light


class Stream:
    """Parses the simulator output, keeping a decimated trail plus the latest tick."""

    def __init__(self):
        self.lock = threading.Lock()
        self.trail = []  # (x, y) in metres
        self.pose = None  # (x, y, theta)
        self.time = 0.0
        self.sensors = []  # IR hits in the robot frame, metres
        self.maze = None  # walls as a [y][x] bool array, one entry per sim grid square
        self.done = False

    def feed(self, line):
        if line.startswith("MAZE:"):
            _, w, h, bits = line.split()
            w, h = int(w), int(h)
            maze = np.frombuffer(bits.encode(), dtype=np.uint8).reshape(h, w) == ord("1")
            with self.lock:
                self.maze = maze
        elif line.startswith("TIME:"):
            t = float(line[5:])
            with self.lock:
                self.time = t
        elif line.startswith("WORLD:"):
            xy, theta = line[6:].split("THETA:")
            x, y = (float(v) for v in xy.split(","))
            theta = float(theta)
            with self.lock:
                self.pose = (x, y, theta)
                if not self.trail or math.dist(self.trail[-1], (x, y)) >= MIN_STEP_METERS:
                    self.trail.append((x, y))
        elif line[:2] == "0:":
            hits = []
            parts = line.replace(",", " ").split()
            for i in range(0, len(parts) - 2, 3):  # ["0:", x, y, "1:", x, y, ...]
                hits.append((float(parts[i + 1]), float(parts[i + 2])))
            with self.lock:
                self.sensors = hits

    def read(self, f):
        for line in f:
            try:
                self.feed(line.strip())
            except ValueError:
                pass  # partial or unexpected line
        self.done = True


def to_world(pose, hit):
    """Robot frame (+y forward, +x right) to world metres."""
    x, y, theta = pose
    a = theta - math.pi / 2
    sx, sy = hit
    return (x + sx * math.cos(a) - sy * math.sin(a), y + sx * math.sin(a) + sy * math.cos(a))


def robot_shape(pose, size=0.35):
    """Triangle pointing along theta, in cell units."""
    x, y, theta = pose[0] / CELL_SIZE_METERS, pose[1] / CELL_SIZE_METERS, pose[2]
    pts = []
    for ang, r in ((0, size), (2.5, size * 0.7), (-2.5, size * 0.7)):
        pts.append((x + r * math.cos(theta + ang), y + r * math.sin(theta + ang)))
    return pts


def setup_axes():
    fig, ax = plt.subplots(figsize=(8, 8))
    for x, y in CENTER_GOAL_CELLS:
        ax.add_patch(Rectangle((x, y), 1, 1, color="#f6d860", alpha=0.5, lw=0))
    ax.add_patch(Rectangle((0, 0), 1, 1, color="#8fd18f", alpha=0.6, lw=0))
    for i in range(N + 1):
        ax.plot([i, i], [0, N], color="#cccccc", lw=0.8, zorder=1)
        ax.plot([0, N], [i, i], color="#cccccc", lw=0.8, zorder=1)
    ax.add_patch(Rectangle((0, 0), N, N, fill=False, lw=2.5, ec="black", zorder=2))
    ax.set_xlim(-0.5, N + 0.5)
    ax.set_ylim(-0.5, N + 0.5)
    ax.set_aspect("equal")
    ax.set_xticks([i + 0.5 for i in range(N)], [str(i) for i in range(N)], fontsize=8)
    ax.set_yticks([i + 0.5 for i in range(N)], [str(i) for i in range(N)], fontsize=8)
    ax.tick_params(length=0)
    for side in ax.spines.values():
        side.set_visible(False)
    fig.tight_layout()
    return fig, ax


def make_artists(ax):
    trail, = ax.plot([], [], color="#1f6fd1", lw=2.5, zorder=3)
    rays = [ax.plot([], [], color="#c0392b", lw=0.8, alpha=0.5, zorder=3)[0] for _ in range(4)]
    hits, = ax.plot([], [], "o", color="#c0392b", ms=4, zorder=4)
    robot = Polygon([(0, 0)] * 3, closed=True, fc="#1a8f1a", ec="black", lw=1, zorder=5)
    robot.set_visible(False)
    ax.add_patch(robot)
    walls = ax.imshow(np.zeros((1, 1, 4)), extent=(0, N, 0, N), origin="lower",
                      interpolation="nearest", zorder=2.5)
    ax.set_xlim(-0.5, N + 0.5)  # imshow resets the limits to its extent
    ax.set_ylim(-0.5, N + 0.5)
    return trail, rays, hits, robot, walls


def update(stream, fig, artists):
    trail, rays, hits, robot, walls = artists
    with stream.lock:
        pts = list(stream.trail)
        pose, sensors, t = stream.pose, list(stream.sensors), stream.time
        maze, stream.maze = stream.maze, None
    if maze is not None:
        rgba = np.zeros(maze.shape + (4,))
        rgba[maze] = (0.15, 0.15, 0.15, 1.0)
        walls.set_data(rgba)
    trail.set_data([p[0] / CELL_SIZE_METERS for p in pts], [p[1] / CELL_SIZE_METERS for p in pts])
    if pose is None:
        return
    robot.set_xy(robot_shape(pose))
    robot.set_visible(True)
    cx, cy = pose[0] / CELL_SIZE_METERS, pose[1] / CELL_SIZE_METERS
    hx, hy = [], []
    for i, ray in enumerate(rays):
        if i >= len(sensors) or not all(map(math.isfinite, sensors[i])):
            ray.set_data([], [])
            continue
        wx, wy = to_world(pose, sensors[i])
        wx, wy = wx / CELL_SIZE_METERS, wy / CELL_SIZE_METERS
        if not (-1 <= wx <= N + 1 and -1 <= wy <= N + 1):  # out of range: no wall seen
            ray.set_data([], [])
            continue
        ray.set_data([cx, wx], [cy, wy])
        hx.append(wx)
        hy.append(wy)
    hits.set_data(hx, hy)
    fig.canvas.manager.set_window_title(f"sim  t = {t:.2f} s")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("file", nargs="?", help="saved simulator output (default: stdin)")
    parser.add_argument("--save", metavar="PNG", help="read to the end, then write an image")
    args = parser.parse_args()

    src = open(args.file) if args.file else sys.stdin
    if src.isatty():
        parser.error("pipe the simulator in, e.g. .pio/build/sim/program | python3 tools/simviz.py")

    if args.save:
        matplotlib.use("Agg")

    stream = Stream()
    fig, ax = setup_axes()
    artists = make_artists(ax)

    if args.save:
        stream.read(src)
        update(stream, fig, artists)
        fig.savefig(args.save, dpi=100)
        return

    threading.Thread(target=stream.read, args=(src,), daemon=True).start()
    anim = FuncAnimation(fig, lambda _: update(stream, fig, artists), interval=100,
                         cache_frame_data=False)
    plt.show()
    del anim


if __name__ == "__main__":
    main()
