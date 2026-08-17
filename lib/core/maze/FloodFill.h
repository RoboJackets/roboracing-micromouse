#pragma once
#include "MazeMap.h"

enum class Reachability { AllCells, KnownCellsOnly };

struct DistanceField {
  int d[N][N];
  int at(Cell c) const { return inBounds(c) ? d[c.y][c.x] : INF; }
};

DistanceField floodFill(const MazeMap &map, const Goals &goal,
                        Reachability reach);
