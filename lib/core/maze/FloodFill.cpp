#include "FloodFill.h"

DistanceField floodFill(const MazeMap &map, const Goals &goal,
                        Reachability reach) {
  DistanceField field{};
  for (int y = 0; y < N; ++y)
    for (int x = 0; x < N; ++x)
      field.d[y][x] = INF;

  Cell queue[N * N];
  int head = 0;
  int tail = 0;

  for (int i = 0; i < goal.count; ++i) {
    const Cell g = goal.cells[i];
    if (!inBounds(g))
      continue;
    field.d[g.y][g.x] = 0;
    queue[tail++] = g;
  }

  while (head < tail) {
    const Cell c = queue[head++];
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
      if (tail < N * N)
        queue[tail++] = n;
    }
  }
  return field;
}
