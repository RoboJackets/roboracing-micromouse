#pragma once
#include "FloodFill.h"


constexpr int BLOCKED = INF + 200; 
constexpr int WORSE_THAN_ANY = INF + 100;

Dir nextStep(const MazeMap &map, const DistanceField &field, Cell at,
             Dir facing, const Goals &goal);
