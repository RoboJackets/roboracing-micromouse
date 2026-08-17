#pragma once
#include "Route.h"

unsigned char exploreStep(const MazeMap &map, Cell at, Dir facing,
                          const Goals &goal, Reachability reach,
                          bool fastSpeed);

unsigned char stepOpcode(Dir facing, Dir target, bool fastSpeed);
