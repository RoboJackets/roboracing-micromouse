#include "Explorer.h"

#include "Commands.h"

unsigned char stepOpcode(Dir facing, Dir target, bool fast) {
  switch ((static_cast<int>(target) - static_cast<int>(facing)) & 3) {
  case 0:
    return fast ? FWD0 + 1 : EX_FWD0 + 1;
  case 1:
    return fast ? ST90R : EX_ST90R;
  case 2:
    return IPT180;
  default:
    return fast ? ST90L : EX_ST90L;
  }
}

unsigned char exploreStep(const MazeMap &map, Cell at, Dir facing,
                          const Goals &goal, bool fast) {
  if (atGoal(at, goal))
    return STOP;

  const DistanceField field = floodFill(
      map, goal, fast ? Reachability::KnownCellsOnly : Reachability::AllCells);
  return stepOpcode(facing, nextStep(map, field, at, facing, goal), fast);
}
