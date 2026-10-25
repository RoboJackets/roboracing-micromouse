#include "Explorer.h"

#include "Commands.h"


Move relativeMove(Dir facing, Dir target) {
  switch ((static_cast<int>(target) - static_cast<int>(facing)) & 3) {
  case 0:
    return Move::Forward;
  case 1:
    return Move::Right;
  case 2:
    return Move::Back;
  default:
    return Move::Left;
  }
}
Move exploreStep(const MazeMap &map, Cell at, Dir facing,
                          const Goals &goal, Reachability reach,
                          bool fastSpeed) {
  if (atGoal(at, goal))
    return Move::Stop;

  const DistanceField field = floodFill(map, goal, reach);
  const Dir d = nextStep(map, field, at, facing, goal);
  return relativeMove(facing, d);
}
