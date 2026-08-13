#pragma once
#include "Route.h"

unsigned char exploreStep(const MazeMap &map, Cell at, Dir facing,
                          const Goals &goal, bool fast);

unsigned char stepOpcode(Dir facing, Dir target, bool fast);
