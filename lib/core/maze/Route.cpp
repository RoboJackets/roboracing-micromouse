#include "Route.h"

namespace {

void neighborCosts(const MazeMap &map, Cell at, const DistanceField &field,
                   int out[4]) {
  for (int i = 0; i < 4; ++i) {
    const Dir d = SCAN_ORDER[i];
    out[i] = map.isOpen(at, d) ? field.at(step(at, d)) : BLOCKED;
  }
}

void applyTiebreaker(const MazeMap &map, const Goals &goal, Cell at, Dir facing,
                     DistanceField &field) {
  for (const Dir d : SCAN_ORDER) {
    if (!map.isOpen(at, d))
      continue;
    const Cell n = step(at, d);
    field.d[n.y][n.x] += goal.turnPenalty * turnCost(facing, d);
  }
  for (int y = 0; y < N; ++y)
    for (int x = 0; x < N; ++x)
      if (!map.isKnown(Cell{x, y}))
        field.d[y][x] -= goal.explorationWeight;
}

} // namespace

Dir nextStep(const MazeMap &map, const DistanceField &field, Cell at,
             Dir facing, const Goals &goal) {
  int costs[4];
  neighborCosts(map, at, field, costs);

  int best = WORSE_THAN_ANY;
  int bestIdx = -1;
  bool tie = false;
  for (int i = 0; i < 4; ++i) {
    if (costs[i] == best)
      tie = true;
    if (costs[i] < best) {
      best = costs[i];
      bestIdx = i;
      tie = false;
    }
  }

  if (tie) {
    DistanceField adjusted = field;
    applyTiebreaker(map, goal, at, facing, adjusted);
    neighborCosts(map, at, adjusted, costs);
    best = WORSE_THAN_ANY;
    bestIdx = -1;
    for (int i = 0; i < 4; ++i) {
      if (costs[i] < best) {
        best = costs[i];
        bestIdx = i;
      }
    }
  }

  return bestIdx < 0 ? facing : SCAN_ORDER[bestIdx];
}
