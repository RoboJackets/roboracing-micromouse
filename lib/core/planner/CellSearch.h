#pragma once
#include <vector>

#include "maze/MazeMap.h"
#include "planner/Commands.h"

std::vector<Move> planFastRoute(const MazeMap &map, const Goals &goal,
                                Cell start = Cell{0, 0},
                                Dir facing = Dir::North);
