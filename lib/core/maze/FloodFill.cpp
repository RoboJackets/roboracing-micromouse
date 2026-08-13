#include "FloodFill.h"

#include <queue>

DistanceField floodFill(const MazeMap &map, const Goals &goal,
                        Reachability reach) {
  DistanceField field{};
  for (int y = 0; y < N; ++y)
    for (int x = 0; x < N; ++x)
      field.d[y][x] = INF;

  std::queue<Cell> queue{};
  for (int i = 0; i < goal.count; ++i) {
    const Cell g = goal.cells[i];
    field.d[g.y][g.x] = 0;
    queue.push(g);
  }

  while (!queue.empty()) {
    const Cell c = queue.front();
    queue.pop();
    const int dist = field.d[c.y][c.x];

    for (const Dir d : SCAN_ORDER) {
      if (!map.isOpen(c, d))
        continue;
      const Cell n = step(c, d);
      if (reach == Reachability::KnownCellsOnly && !map.isKnown(n))
        continue;
      if (dist + 1 >= field.d[n.y][n.x])
        continue;
      field.d[n.y][n.x] = dist + 1;
      queue.push(n);
    }
  }
  return field;
}
