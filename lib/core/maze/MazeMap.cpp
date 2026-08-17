#include "MazeMap.h"

#include <cmath>

#include "Tuning.h"

bool MazeMap::hasWall(Cell c, Dir d) const {
  if (!inBounds(c))
    return true;
  return walls[c.y][c.x] & wallBit(d);
}

void MazeMap::addWall(Cell c, Dir d) {
  if (!inBounds(c))
    return;
  walls[c.y][c.x] |= wallBit(d);
  const Cell n = step(c, d);
  if (!inBounds(n))
    return;
  walls[n.y][n.x] |= wallBit(opposite(d));
}

bool MazeMap::isKnown(Cell c) const {
  return inBounds(c) && explored[c.y][c.x];
}

void MazeMap::markKnown(Cell c) {
  if (inBounds(c))
    explored[c.y][c.x] = true;
}

bool MazeMap::isOpen(Cell c, Dir d) const {
  return inBounds(step(c, d)) && !hasWall(c, d);
}

void MazeMap::addBorderWalls() {
  for (int i = 0; i < N; ++i) {
    walls[i][0] |= LEFT;
    walls[0][i] |= DOWN;
    walls[i][N - 1] |= RIGHT;
    walls[N - 1][i] |= TOP;
  }
}

// adds walls to the maze map.
void MazeMap::observe(const WorldCoord &pose, double rotationRate,
                      const std::array<WorldCoord, 4> &readings) {
  if (!updatesEnabled)
    return;
  const Cell at = cellOf(pose);
  if (!inBounds(at))
    return;
  markKnown(at);

  if (std::abs(std::remainder(pose.theta, M_PI / 2.0)) > OBSERVE_MAX_HEADING_ERROR)
    return;
  if (std::abs(rotationRate) > OBSERVE_MAX_ROTATION_RATE)
    return;

  const Dir facing = dirOf(pose.theta);
  const WorldCoord rel = cellRelative(pose, at);

  switch (facing) {
  case Dir::North:
    if (rel.y > OBSERVE_ENTRY_MARGIN)
      return;
    break;
  case Dir::South:
    if (rel.y < OBSERVE_ENTRY_MARGIN)
      return;
    break;
  case Dir::West:
    if (rel.x < OBSERVE_ENTRY_MARGIN)
      return;
    break;
  case Dir::East:
    if (rel.x > OBSERVE_ENTRY_MARGIN)
      return;
    break;
  }

  if (readings[0].y < WALL_THRESHOLD_FRONT)
    addWall(at, facing);
  if (-readings[2].x < WALL_THRESHOLD_LEFT)
    addWall(at, turnLeft(facing));
  if (readings[3].x < WALL_THRESHOLD_RIGHT)
    addWall(at, turnRight(facing));
}
