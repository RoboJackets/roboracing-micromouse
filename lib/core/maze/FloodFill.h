#pragma once
#include "MazeMap.h"

enum class Reachability { AllCells, KnownCellsOnly };

struct DistanceField {
  int d[N][N];
  int at(Cell c) const { return d[c.y][c.x]; }
};

DistanceField floodFill(const MazeMap &map, const Goals &goal,
                        Reachability reach);
