#pragma once
#include <vector>

#include "maze/MazeMap.h"

std::vector<unsigned char> planFastRoute(const MazeMap &map, const Goals &goal,
                                         Cell start = Cell{0, 0},
                                         Dir facing = Dir::North);
