#pragma once
#include <vector>

#include "maze/MazeMap.h"
std::vector<unsigned char> planFastRoute(const MazeMap &map, const Goals &goal);
