#pragma once
#include "Route.h"
#include "planner/Commands.h"
Move exploreStep(const MazeMap &map, Cell at, Dir facing, const Goals &goal,
                 Reachability reach, bool fastSpeed);

Move relativeMove(Dir facing, Dir target);
