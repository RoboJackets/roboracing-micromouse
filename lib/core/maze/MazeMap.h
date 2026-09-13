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

  void observe(const WorldCoord &pose, double rotationRate,
               const std::array<WorldCoord, 4> &readings);
};
