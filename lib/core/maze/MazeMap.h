#pragma once
#include <array>

#include "Maze.h"

struct MazeMap {

  unsigned char walls[N][N]{};
  bool explored[N][N]{};
  bool updatesEnabled = false;

  bool hasWall(Cell c, Dir d) const;

  void addWall(Cell c, Dir d);
  bool isKnown(Cell c) const;
  void markKnown(Cell c);
  bool isOpen(Cell c, Dir d) const;
  void addBorderWalls();

  void allowUpdates(bool enabled) { updatesEnabled = enabled; }

  void observe(Cell at, const Pose2d &pose, double rotationRate,
               const std::array<Translation2d, 4> &readings);
};
