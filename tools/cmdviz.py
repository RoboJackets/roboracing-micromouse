#!/usr/bin/env python3
"""Visualize the path described by cmdgen's binary Command output on a 16x16 maze.

Usage:
    echo FFRFFLFS | .pio/build/cmdgen/program | python3 tools/cmdviz.py
    python3 tools/cmdviz.py commands.bin
    ... | python3 tools/cmdviz.py --save path.png    # write an image instead of opening a window

The robot starts in the center of cell (0, 0) (bottom-left) facing north. Each
Forward moves N cells; each 90° SmoothTurn rotates and moves one cell, drawn as a
quarter arc through the cell where the heading changes.
"""
import argparse
import math
import struct
import sys

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

N = 16
CENTER_GOAL_CELLS = [(7, 7), (7, 8), (8, 7), (8, 8)]

# Must match `struct Command` in lib/core/planner/CommandGenerator.h as dumped
# by src/cmdgen/main.cpp: 4-byte enum `type`, int8_t `amt`, 3 bytes padding.
RECORD = struct.Struct("<ib3x")
FORWARD, SMOOTH_TURN, STOP = 0, 1, 2
TYPE_NAMES = {FORWARD: "Forward", SMOOTH_TURN: "SmoothTurn", STOP: "Stop"}

# Headings in clockwise order so a positive (right) turn increments the index.
HEADINGS = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # N, E, S, W

# Cycled per command so adjacent segments are easy to tell apart.
SEGMENT_COLORS = ["#1f6fd1", "#e67e22", "#16a085", "#8e44ad", "#c0392b"]


def decode(data):
    if len(data) % RECORD.size:
        sys.exit(f"error: got {len(data)} bytes, not a multiple of the "
                 f"{RECORD.size}-byte Command record")
    return [RECORD.unpack_from(data, i) for i in range(0, len(data), RECORD.size)]


def describe(cmd):
    type_, amt = cmd
    if type_ == FORWARD:
        return f"FWD {amt}"
    if type_ == SMOOTH_TURN:
        return f"TURN {'R' if amt > 0 else 'L'} {abs(amt) * 45}°"
    if type_ == STOP:
        return "STOP"
    return f"UNKNOWN(type={type_}, amt={amt})"


def simulate(commands):
    """Return (cells visited in order, final heading index, spans, warnings).

    spans[i] = (first, last) cell-step indices covered by commands[i], for each
    command that was simulated; step k goes from cells[k] to cells[k + 1].
    """
    cell = (0, 0)
    heading = 0
    cells = [cell]
    spans = []
    warnings = []

    def advance(n):
        nonlocal cell
        dx, dy = HEADINGS[heading]
        for _ in range(n):
            cell = (cell[0] + dx, cell[1] + dy)
            cells.append(cell)

    for i, (type_, amt) in enumerate(commands):
        first = len(cells) - 1
        if type_ == FORWARD:
            advance(amt)
        elif type_ == SMOOTH_TURN:
            if amt % 2:
                warnings.append(f"#{i}: {abs(amt) * 45}° turn (diagonal) not drawn")
                break
            heading = (heading + amt // 2) % 4
            advance(1)
        elif type_ == STOP:
            spans.append((first, first))
            if i != len(commands) - 1:
                warnings.append(f"#{i}: commands after STOP ignored")
            break
        else:
            warnings.append(f"#{i}: unknown command type {type_}")
            break
        spans.append((first, len(cells) - 1))
    else:
        if commands:
            warnings.append("path does not end with STOP")

    off = [c for c in cells if not (0 <= c[0] < N and 0 <= c[1] < N)]
    if off:
        warnings.append(f"path leaves the {N}x{N} maze at {off[0]}")
    return cells, heading, spans, warnings


def step_pieces(cells, samples=12):
    """Return one polyline per cell step, with 90° corners drawn as quarter arcs.

    A corner arc (radius half a cell) belongs to the step that turns, so it begins
    on the edge where the robot enters the corner cell. Joining consecutive pieces
    gives one continuous path.
    """
    centers = [(x + 0.5, y + 0.5) for x, y in cells]
    n = len(centers) - 1

    def d(k):
        return (centers[k + 1][0] - centers[k][0], centers[k + 1][1] - centers[k][1])

    def corner(k):
        return 0 < k < n and d(k - 1) != d(k) and d(k - 1) != (-d(k)[0], -d(k)[1])

    pieces = []
    for k in range(n):
        cur, nxt = centers[k], centers[k + 1]
        if corner(k):
            din, dout = d(k - 1), d(k)
            p1 = (cur[0] - din[0] / 2, cur[1] - din[1] / 2)
            p2 = (cur[0] + dout[0] / 2, cur[1] + dout[1] / 2)
            c = (p1[0] + dout[0] / 2, p1[1] + dout[1] / 2)
            a1 = math.atan2(p1[1] - c[1], p1[0] - c[0])
            a2 = math.atan2(p2[1] - c[1], p2[0] - c[0])
            sweep = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi  # shortest, ±90°
            pts = [(c[0] + 0.5 * math.cos(a1 + sweep * i / samples),
                    c[1] + 0.5 * math.sin(a1 + sweep * i / samples))
                   for i in range(samples + 1)]
        else:
            pts = [cur]
        # Stop on the shared edge if the next cell starts a corner, else at its center.
        if corner(k + 1):
            pts.append((nxt[0] - d(k)[0] / 2, nxt[1] - d(k)[1] / 2))
        else:
            pts.append(nxt)
        pieces.append(pts)
    return pieces


def draw(cells, heading, spans, save_path=None):
    if save_path:
        matplotlib.use("Agg")

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title("micromouse path visualizer")

    for x, y in CENTER_GOAL_CELLS:
        ax.add_patch(Rectangle((x, y), 1, 1, color="#f6d860", alpha=0.5, lw=0))
    ax.add_patch(Rectangle((0, 0), 1, 1, color="#8fd18f", alpha=0.6, lw=0))
    for i in range(N + 1):
        ax.plot([i, i], [0, N], color="#cccccc", lw=0.8, zorder=1)
        ax.plot([0, N], [i, i], color="#cccccc", lw=0.8, zorder=1)
    ax.add_patch(Rectangle((0, 0), N, N, fill=False, lw=2.5, ec="black", zorder=2))

    pieces = step_pieces(cells)
    start = (cells[0][0] + 0.5, cells[0][1] + 0.5)
    end = (cells[-1][0] + 0.5, cells[-1][1] + 0.5)

    # One colored segment per command, with a boundary marker where it begins.
    for i, (first, last) in enumerate(spans):
        if first == last:
            continue  # STOP or FWD 0: no movement
        color = SEGMENT_COLORS[i % len(SEGMENT_COLORS)]
        pts = [p for piece in pieces[first:last] for p in piece]
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color=color, lw=3.5, solid_capstyle="butt", zorder=3)
        if first > 0:
            ax.plot(*pts[0], "o", ms=7, mfc="white", mec="#333333", mew=1.5, zorder=5)

    ax.plot(*start, "o", color="#1a8f1a", ms=12, zorder=5)
    dx, dy = HEADINGS[heading]
    ax.annotate("", xy=(end[0] + 0.35 * dx, end[1] + 0.35 * dy), xytext=end,
                arrowprops=dict(arrowstyle="-|>", color="#c0392b", lw=2.5,
                                mutation_scale=22), zorder=5)
    for x, y in cells:
        if not (0 <= x < N and 0 <= y < N):
            ax.plot(x + 0.5, y + 0.5, "x", color="red", ms=10, mew=3, zorder=5)

    ax.set_xlim(-0.5, N + 0.5)
    ax.set_ylim(-0.5, N + 0.5)
    ax.set_aspect("equal")
    ax.set_xticks([i + 0.5 for i in range(N)], [str(i) for i in range(N)], fontsize=8)
    ax.set_yticks([i + 0.5 for i in range(N)], [str(i) for i in range(N)], fontsize=8)
    ax.tick_params(length=0)
    for side in ax.spines.values():
        side.set_visible(False)
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=100)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("file", nargs="?", help="binary command file (default: stdin)")
    parser.add_argument("--save", metavar="PNG", help="write an image instead of opening a window")
    args = parser.parse_args()

    if args.file:
        with open(args.file, "rb") as f:
            data = f.read()
    else:
        if sys.stdin.isatty():
            parser.error("pipe cmdgen output in, e.g. "
                         "echo FFRS | .pio/build/cmdgen/program | python3 tools/cmdviz.py")
        data = sys.stdin.buffer.read()

    commands = decode(data)
    cells, heading, spans, warnings = simulate(commands)
    for line in map(describe, commands):
        print(line)
    for w in warnings:
        print(f"warning: {w}", file=sys.stderr)
    draw(cells, heading, spans, args.save)


if __name__ == "__main__":
    main()
