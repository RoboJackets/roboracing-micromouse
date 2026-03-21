#!/usr/bin/env python3
"""
generate_test_log.py
====================
Generates a test CSV for the analyzer-ui that exercises:
  - Accurate robot pose on the 16×16 maze grid
  - Cells lighting up (explored[i][j]) as the robot visits them
  - Walls appearing (walls[i][j]) as they are discovered

Usage:
    python3 generate_test_log.py
Output:
    test_maze.csv  (load this in the analyzer-ui)
"""

import csv
import math
import os

# ── Constants (must match firmware) ─────────────────────────────────────────
CELL_M        = 0.2          # CELL_SIZE_METERS
N             = 16           # Grid dimension
TICK_US       = 100_000      # 100 ms per row
TICKS_PER_CELL = 25          # rows of smooth interpolation per cell transition

# Wall bitmasks (mirror Types.h)
TOP   = 0b1000   # 8
RIGHT = 0b0100   # 4
DOWN  = 0b0010   # 2
LEFT  = 0b0001   # 1

DIR_ANGLE = {
    ( 1,  0): 0.0,
    (-1,  0): math.pi,
    ( 0,  1): math.pi / 2,
    ( 0, -1): 3 * math.pi / 2,
}

WALL_FOR_MOVE = {
    ( 1,  0): RIGHT,
    (-1,  0): LEFT,
    ( 0,  1): TOP,
    ( 0, -1): DOWN,
}

OPPOSITE = { TOP: DOWN, DOWN: TOP, LEFT: RIGHT, RIGHT: LEFT }

NEIGHBOR = {
    TOP:   ( 0,  1),
    DOWN:  ( 0, -1),
    LEFT:  (-1,  0),
    RIGHT: ( 1,  0),
}


# ── Ground-truth maze definition ─────────────────────────────────────────────
def build_maze():
    """
    A hand-crafted 16×16 maze layout.
    walls[x][y] is a bitmask of walls on that cell.
    Outer boundary is always walled.  A few interior passages are blocked.
    """
    w = [[0] * N for _ in range(N)]

    # Outer boundary
    for i in range(N):
        w[i][0]     |= DOWN
        w[i][N - 1] |= TOP
        w[0][i]     |= LEFT
        w[N - 1][i] |= RIGHT

    # Helper to place a wall segment and its symmetric opposite
    def wall(x, y, side):
        if 0 <= x < N and 0 <= y < N:
            w[x][y] |= side
        nx, ny = x + NEIGHBOR[side][0], y + NEIGHBOR[side][1]
        if 0 <= nx < N and 0 <= ny < N:
            w[nx][ny] |= OPPOSITE[side]

    # --- Interior walls (defines corridors through the first 8×8 corner) ---
    # Horizontal run: y=4 from x=1..4
    for x in range(1, 5):
        wall(x, 4, DOWN)

    # Vertical run: x=4 from y=1..3
    for y in range(1, 4):
        wall(4, y, RIGHT)

    # U-bend at the top of column 2
    wall(2, 6, TOP)
    wall(2, 6, RIGHT)
    wall(3, 6, TOP)

    # Dead-end pocket at (6,2)
    wall(6, 2, TOP)
    wall(6, 2, RIGHT)
    wall(6, 3, RIGHT)

    # Long horizontal barrier at y=8 from x=1..7
    for x in range(1, 8):
        wall(x, 8, DOWN)

    return w


# ── Path: snake through the first 8 columns × 8 rows ─────────────────────────
def generate_path():
    """
    Visit cells in a snake pattern:
      col 0 bottom→top, col 1 top→bottom, col 2 bottom→top, …
    covering an 8×8 sub-grid so the run completes in reasonable time.
    """
    cells = []
    for col in range(8):
        if col % 2 == 0:
            rows = range(0, 8)
        else:
            rows = range(7, -1, -1)
        for row in rows:
            cells.append((col, row))
    return cells


# ── Angle helpers ─────────────────────────────────────────────────────────────
def lerp(a, b, t):
    return a + (b - a) * t


def lerp_angle(a, b, t):
    d = (b - a + math.pi) % (2 * math.pi) - math.pi
    return a + d * t


# ── Noise helper (deterministic, no random seed needed) ──────────────────────
def pseudo_noise(seed, scale=5.0):
    return (((seed * 1664525 + 1013904223) & 0xFFFFFFFF) / 0xFFFFFFFF - 0.5) * scale


# ── Main generator ────────────────────────────────────────────────────────────
def generate():
    maze = build_maze()
    path = generate_path()

    # Runtime state
    disc_walls  = [[0]    * N for _ in range(N)]
    explored    = [[False] * N for _ in range(N)]

    rows = []
    t_us = 0
    prev_theta = math.pi / 2   # start facing north

    for step, (cx, cy) in enumerate(path):
        # ── Mark current cell explored + discover its walls ───────────────
        explored[cx][cy] = True
        cell_walls = maze[cx][cy]
        disc_walls[cx][cy] |= cell_walls
        for side in (TOP, RIGHT, DOWN, LEFT):
            if cell_walls & side:
                nx2 = cx + NEIGHBOR[side][0]
                ny2 = cy + NEIGHBOR[side][1]
                if 0 <= nx2 < N and 0 <= ny2 < N:
                    disc_walls[nx2][ny2] |= OPPOSITE[side]

        # ── Movement direction to next cell ───────────────────────────────
        if step + 1 < len(path):
            nxt_x, nxt_y = path[step + 1]
        else:
            nxt_x, nxt_y = cx, cy
        dx, dy = nxt_x - cx, nxt_y - cy
        target_theta = DIR_ANGLE.get((dx, dy), prev_theta)

        # World coords: center of current vs next cell
        wx0 = cx * CELL_M + CELL_M / 2
        wy0 = cy * CELL_M + CELL_M / 2
        wx1 = nxt_x * CELL_M + CELL_M / 2
        wy1 = nxt_y * CELL_M + CELL_M / 2

        # ── Emit TICKS_PER_CELL rows for smooth animation ─────────────────
        for tick in range(TICKS_PER_CELL):
            f = tick / TICKS_PER_CELL               # 0 → 1 across the cell

            # Position: linear
            wx = lerp(wx0, wx1, f)
            wy = lerp(wy0, wy1, f)

            # Heading: fast turn at start of movement
            wtheta = lerp_angle(prev_theta, target_theta, min(1.0, f * 4))

            # Simulated IR readings (plausible distance-to-wall values)
            ir = []
            for si in range(4):
                base  = 60.0 + 40.0 * (1 - f) * (si % 2 == 0)
                noise = pseudo_noise(step * 1000 + tick * 10 + si)
                ir.append(round(max(30.0, min(200.0, base + noise)), 2))

            # Current grid state
            move_dir_bits = WALL_FOR_MOVE.get((dx, dy), 0)

            row = {
                "timestamp_us": t_us,
                "w/x":          f"{wx:.4f}",
                "w/y":          f"{wy:.4f}",
                "w/theta":      f"{wtheta:.4f}",
                "IR[0]":        ir[0],
                "IR[1]":        ir[1],
                "IR[2]":        ir[2],
                "IR[3]":        ir[3],
                "state":        step % 4,
                "state/x":      cx,
                "state/y":      cy,
                "state/dir":    move_dir_bits,
            }

            # 256 wall columns  (walls[x][y])
            for i in range(N):
                for j in range(N):
                    row[f"walls[{i}][{j}]"]    = disc_walls[i][j]
                    row[f"explored[{i}][{j}]"] = 1 if explored[i][j] else 0

            rows.append(row)
            t_us += TICK_US

        prev_theta = target_theta

    return rows


# ── Write CSV ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Generating test log…")
    rows = generate()

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_maze.csv")
    fieldnames = list(rows[0].keys())

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    total_time_s = rows[-1]["timestamp_us"] / 1_000_000
    print(f"  Wrote {len(rows):,} rows × {len(fieldnames)} columns → {out_path}")
    print(f"  Simulated duration: {total_time_s:.1f} s")
    print(f"  Load 'test_maze.csv' in the analyzer-ui to verify pose + maze discovery.")
