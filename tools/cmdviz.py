#!/usr/bin/env python3
"""Visualize the path described by cmdgen's binary Command output on a 16x16 maze.

Usage:
    echo FFRFFLFS | .pio/build/cmdgen/program | python3 tools/cmdviz.py
    python3 tools/cmdviz.py commands.bin
    python3 tools/cmdviz.py --moves FFRLRL
    ... | python3 tools/cmdviz.py --save path.png    # write an image instead of opening a window

The robot starts in the center of cell (0, 0) (bottom-left) facing north. Forward
moves along the current heading; each SmoothTurn rotates by ``amt * 45°`` and
moves one cell. Right-angle turns are drawn as quarter arcs; diagonal legs are
drawn as straight lines. Use ``--moves`` to interactively step through how input
instructions are converted into commands.
"""
import argparse
import math
import struct
import sys
import textwrap

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.widgets import Button

N = 16
CENTER_GOAL_CELLS = [(7, 7), (7, 8), (8, 7), (8, 8)]

# Must match `struct Command` in lib/core/planner/CommandGenerator.h as dumped
# by src/cmdgen/main.cpp: 4-byte enum `type`, int8_t `amt`, 3 bytes padding.
RECORD = struct.Struct("<ib3x")
FORWARD, SMOOTH_TURN, STOP, IN_PLACE_TURN = 0, 1, 2, 3
TYPE_NAMES = {FORWARD: "Forward", SMOOTH_TURN: "SmoothTurn", STOP: "Stop",
              IN_PLACE_TURN: "InPlaceTurn"}

# Octant offsets in clockwise order so a positive (right) turn increments index.
HEADINGS = [
    (0, 1), (1, 1), (1, 0), (1, -1),
    (0, -1), (-1, -1), (-1, 0), (-1, 1),
]  # N, NE, E, SE, S, SW, W, NW
MOVE_NAMES = {"F": "Forward", "L": "Left", "R": "Right", "B": "Back", "S": "Stop"}

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
    if type_ == IN_PLACE_TURN:
        return f"IPT {abs(amt) * 45}°"
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
            heading = (heading + amt) % len(HEADINGS)
            advance(1)
        elif type_ == IN_PLACE_TURN:
            heading = (heading + amt) % 4
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


def generate_move(state, move):
    """Mirror one CommandGenerator transition; return its new state and output."""
    kind = state[0]
    emitted = []
    if kind == "straight":
        cells = state[1]
        if move == "F":
            return ("straight", cells + 1), emitted
        if move in ("L", "R"):
            if cells:
                emitted.append((FORWARD, cells))
            return ("turning", -1 if move == "L" else 1), emitted
        if move == "S":
            if cells:
                emitted.append((FORWARD, cells))
            emitted.append((STOP, 0))
            return ("straight", 0), emitted
    elif kind == "turning":
        side = state[1]
        if move == "F":
            emitted.append((SMOOTH_TURN, 2 * side))
            return ("straight", 1), emitted
        if move == "L" and side == -1 or move == "R" and side == 1:
            emitted.append((SMOOTH_TURN, 2 * side))
            return state, emitted
        if move in ("L", "R"):
            return ("diagonal", -side, side, 1), emitted
        if move == "S":
            emitted.extend(((SMOOTH_TURN, 2 * side), (STOP, 0)))
            return ("straight", 0), emitted
    elif kind == "diagonal":
        last_side, general_side, cells = state[1:]
        if move == "F":
            emitted.extend(((SMOOTH_TURN, general_side),
                            (FORWARD, cells),
                            (SMOOTH_TURN, last_side)))
            return ("straight", 1), emitted
        if last_side == -1 and move == "R":
            return ("diagonal", 1, general_side, cells + 1), emitted
        if last_side == 1 and move == "L":
            return ("diagonal", -1, general_side, cells), emitted
        if move == ("L" if last_side == -1 else "R"):
            emitted.extend(((SMOOTH_TURN, general_side),
                            (FORWARD, cells),
                            (SMOOTH_TURN, 3 * last_side)))
            return ("straight", 1), emitted
        if move == "S":
            emitted.extend(((SMOOTH_TURN, 2 * last_side), (STOP, 0)))
            return ("straight", 0), emitted
    return state, emitted


def trace_moves(moves):
    """Return command-generator state and emitted commands after each move."""
    state = ("straight", 0)
    commands = []
    trace = [{"move": None, "state": state, "emitted": [], "commands": []}]
    for move in moves:
        state, emitted = generate_move(state, move)
        commands.extend(emitted)
        trace.append({"move": move, "state": state, "emitted": emitted,
                      "commands": commands.copy()})
    return trace


def describe_state(state):
    if state[0] == "straight":
        return f"Straight; {state[1]} forward move(s) buffered"
    if state[0] == "turning":
        return f"Turning {'right' if state[1] > 0 else 'left'}; waiting for resolution"
    _, last_side, general_side, cells = state
    return (f"Diagonal; last side {'right' if last_side > 0 else 'left'}, "
            f"general side {'right' if general_side > 0 else 'left'}, {cells} cell(s)")


def step_pieces(cells, samples=12):
    """Return one polyline per step, with cardinal 90° corners drawn as arcs.

    A corner arc (radius half a cell) belongs to the step that turns, so it begins
    on the edge where the robot enters the corner cell. Diagonal legs remain
    straight. Joining consecutive pieces gives one continuous path.
    """
    centers = [(x + 0.5, y + 0.5) for x, y in cells]
    n = len(centers) - 1

    def d(k):
        return (centers[k + 1][0] - centers[k][0], centers[k + 1][1] - centers[k][1])

    def corner(k):
        if not 0 < k < n:
            return False
        din, dout = d(k - 1), d(k)
        cardinal = lambda direction: direction[0] == 0 or direction[1] == 0
        return (cardinal(din) and cardinal(dout)
                and din[0] * dout[0] + din[1] * dout[1] == 0)

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


def draw_path(ax, cells, heading, spans):
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
    length = math.hypot(dx, dy)
    ax.annotate("", xy=(end[0] + 0.35 * dx / length,
                        end[1] + 0.35 * dy / length), xytext=end,
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


def draw(cells, heading, spans, save_path=None):
    if save_path:
        matplotlib.use("Agg")

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title("micromouse path visualizer")
    draw_path(ax, cells, heading, spans)
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=100)
    else:
        plt.show()


def draw_stepper(moves):
    trace = trace_moves(moves)
    fig, ax = plt.subplots(figsize=(10, 9))
    fig.canvas.manager.set_window_title("micromouse instruction stepper")
    fig.subplots_adjust(top=0.68, bottom=0.16)
    fig.text(0.5, 0.94, f"Instructions: {' '.join(moves)}",
             ha="center", va="top", fontsize=14, family="monospace")
    detail = fig.text(0.5, 0.86, "", ha="center", va="top",
                      fontsize=10, family="monospace")
    step_label = fig.text(0.5, 0.72, "", ha="center", va="top", fontsize=10)
    index = [0]

    def update():
        step = index[0]
        entry = trace[step]
        commands = entry["commands"]
        cells, heading, spans, warnings = simulate(commands)
        ax.clear()
        draw_path(ax, cells, heading, spans)
        if step == 0:
            instruction = "Start"
            emitted = "—"
        else:
            instruction = f"{step}. {entry['move']} ({MOVE_NAMES[entry['move']]})"
            emitted = ", ".join(map(describe, entry["emitted"])) or "none"
        generated = ", ".join(map(describe, commands)) or "none yet"
        detail.set_text(textwrap.fill(
            f"Instruction: {instruction} | Emitted now: {emitted} | "
            f"State: {describe_state(entry['state'])} | Commands so far: {generated}",
            width=110))
        step_label.set_text(f"Step {step} of {len(moves)}")
        path_warnings = [warning for warning in warnings
                         if warning != "path does not end with STOP"]
        if path_warnings:
            step_label.set_text(f"Step {step} of {len(moves)} — {path_warnings[0]}")
        fig.canvas.draw_idle()

    previous_ax = fig.add_axes([0.35, 0.045, 0.13, 0.06])
    next_ax = fig.add_axes([0.52, 0.045, 0.13, 0.06])
    previous_button = Button(previous_ax, "Previous")
    next_button = Button(next_ax, "Next")

    def move_step(delta):
        index[0] = max(0, min(len(moves), index[0] + delta))
        update()

    previous_button.on_clicked(lambda _event: move_step(-1))
    next_button.on_clicked(lambda _event: move_step(1))
    fig.canvas.mpl_connect("key_press_event", lambda event: (
        move_step(-1) if event.key in ("left", "backspace")
        else move_step(1) if event.key in ("right", "enter", " ")
        else None
    ))
    update()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("file", nargs="?", help="binary command file (default: stdin)")
    parser.add_argument("--moves", help="step through move instructions (F, L, R, B, S)")
    parser.add_argument("--save", metavar="PNG", help="write an image instead of opening a window")
    args = parser.parse_args()

    if args.moves is not None:
        if args.file:
            parser.error("--moves cannot be combined with a binary command file")
        moves = "".join(args.moves.upper().split())
        invalid = next((move for move in moves if move not in MOVE_NAMES), None)
        if invalid is not None:
            parser.error(f"unknown move {invalid!r}; expected only F, L, R, B, S")
        if args.save:
            commands = trace_moves(moves)[-1]["commands"]
            cells, heading, spans, warnings = simulate(commands)
            for warning in warnings:
                print(f"warning: {warning}", file=sys.stderr)
            draw(cells, heading, spans, args.save)
        else:
            draw_stepper(moves)
        return

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
