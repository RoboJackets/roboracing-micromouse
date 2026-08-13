#include "MazeMap.h"

#include <cmath>

bool MazeMap::hasWall(Cell c, Dir d) const {
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

bool MazeMap::isKnown(Cell c) const { return explored[c.y][c.x]; }

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
  const Cell at = cellOf(pose);
  if (!inBounds(at))
    return;
  markKnown(at);

  if (std::abs(std::remainder(pose.theta, M_PI / 2.0)) > 0.2)
    return;
  if (std::abs(rotationRate) > 0.15)
    return;
  if (!updatesEnabled)
    return;

  const Dir facing = dirOf(pose.theta);
  const WorldCoord rel = cellRelative(pose, at);

  switch (facing) {
  case Dir::North:
    if (rel.y > 0.1)
      return;
    break;
  case Dir::South:
    if (rel.y < 0.1)
      return;
    break;
  case Dir::West:
    if (rel.x < 0.1)
      return;
    break;
  case Dir::East:
    if (rel.x > 0.1)
      return;
    break;
  }

  if (readings[0].y < 0.075)
    addWall(at, facing);
  if (-readings[2].x < 0.12)
    addWall(at, turnLeft(facing));
  if (readings[3].x < 0.11)
    addWall(at, turnRight(facing));
}
