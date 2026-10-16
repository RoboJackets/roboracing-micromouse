#!/usr/bin/env python3
"""Play back the simulator's pose and IR readings on a 16x16 maze.

The simulator runs much faster than real time, so the window replays the run by
the TIME stamps in its output rather than drawing blocks as they arrive.

Usage:
    .pio/build/sim/program | python3 tools/simviz.py              # play back at real speed
    python3 tools/simviz.py sim.log --speed 4                     # replay a saved run at 4x
    python3 tools/simviz.py sim.log --save run.png                # static image of the whole run

Reads the maze printed once at startup, then the blocks printed every tick:
    MAZE: <w> <h> <w*h 0/1 chars, row by row from y = 0>   (SimIO::printMaze)
    TIME: <seconds since start>
    WORLD: <x>, <y> THETA: <theta>
    0: <x>, <y>     1: <x>, <y>     2: ...      (IR hits, robot frame, metres)
Any other line (e.g. MouseIO::printf output) is echoed to stdout when playback
reaches the sim time it was printed at.
"""
import argparse
import bisect
import math
import sys
import threading
import time

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
    """Parses the simulator output into one frame per printed block."""

    def __init__(self):
        self.lock = threading.Lock()
        self.trail = []  # (x, y) in metres, decimated over the whole run
        self.times = []  # sim time of each frame, ascending
        self.frames = []  # (pose, sensors, trail length) per frame
        self.logs = []  # (sim time, text) of lines that aren't part of a block
        self.maze = None  # walls as a [y][x] bool array, one entry per sim grid square
        self.done = False
        self._time = 0.0  # TIME and WORLD of the block being parsed
        self._pose = None

    def feed(self, raw):
        line = raw.strip()
        if line.startswith("MAZE:"):
            _, w, h, bits = line.split()
            w, h = int(w), int(h)
            maze = np.frombuffer(bits.encode(), dtype=np.uint8).reshape(h, w) == ord("1")
            with self.lock:
                self.maze = maze
        elif line.startswith("TIME:"):
            self._time = float(line[5:])
        elif line.startswith("WORLD:"):
            xy, theta = line[6:].split("THETA:")
            x, y = (float(v) for v in xy.split(","))
            self._pose = (x, y, float(theta))
        elif line[:2] == "0:" and self._pose is not None:  # last line of a block
            hits = []
            parts = line.replace(",", " ").split()
            for i in range(0, len(parts) - 2, 3):  # ["0:", x, y, "1:", x, y, ...]
                hits.append((float(parts[i + 1]), float(parts[i + 2])))
            xy = self._pose[:2]
            with self.lock:
                if not self.trail or math.dist(self.trail[-1], xy) >= MIN_STEP_METERS:
                    self.trail.append(xy)
                self.times.append(self._time)
                self.frames.append((self._pose, hits, len(self.trail)))
        else:
            with self.lock:
                self.logs.append((self._time, raw.rstrip("\r\n")))

    def read(self, f):
        for line in f:
            try:
                self.feed(line)
            except ValueError:
                pass  # partial or unexpected line
        self.done = True


class Echo:
    """Prints the stream's log lines up to a sim time, each once."""

    def __init__(self):
        self.printed = 0

    def upto(self, stream, t=math.inf):
        with stream.lock:
            new = []
            while self.printed < len(stream.logs) and stream.logs[self.printed][0] <= t:
                new.append(stream.logs[self.printed][1])
                self.printed += 1
        if new:
            sys.stdout.write("\n".join(new) + "\n")
            sys.stdout.flush()


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


class Clock:
    """Playback time: sim seconds since the first frame, scaled by speed."""

    def __init__(self, speed):
        self.speed = speed
        self.start = None  # wall time when playback began

    def now(self, stream):
        with stream.lock:
            if not stream.times:
                return None
            first = stream.times[0]
        if self.start is None:
            self.start = time.monotonic()
        return first + (time.monotonic() - self.start) * self.speed


def update(stream, fig, artists, t=math.inf):
    """Draw the last frame at or before sim time t."""
    trail, rays, hits, robot, walls = artists
    with stream.lock:
        maze, stream.maze = stream.maze, None
        i = bisect.bisect_right(stream.times, t) - 1
        if i >= 0:
            pose, sensors, n = stream.frames[i]
            t = stream.times[i]
            pts = stream.trail[:n]
    if maze is not None:
        rgba = np.zeros(maze.shape + (4,))
        rgba[maze] = (0.15, 0.15, 0.15, 1.0)
        walls.set_data(rgba)
    if i < 0:
        return
    trail.set_data([p[0] / CELL_SIZE_METERS for p in pts], [p[1] / CELL_SIZE_METERS for p in pts])
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
    parser.add_argument("--speed", type=float, default=1.0,
                        help="playback speed, sim seconds per real second (default: 1)")
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
        Echo().upto(stream)
        update(stream, fig, artists)
        fig.savefig(args.save, dpi=100)
        return

    threading.Thread(target=stream.read, args=(src,), daemon=True).start()
    clock = Clock(args.speed)
    echo = Echo()

    def tick(_):
        t = clock.now(stream)
        if t is not None:
            echo.upto(stream, t)
            update(stream, fig, artists, t)

    anim = FuncAnimation(fig, tick, interval=33, cache_frame_data=False)
    plt.show()
    del anim


if __name__ == "__main__":
    main()
