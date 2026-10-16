#include "CellSearch.h"

#include <limits>
#include <unordered_set>

#include "maze/Explorer.h"

namespace {

struct CellHash {
  std::size_t operator()(const Cell &c) const { return c.x * N + c.y; }
};

void dfs(const MazeMap &map, const Goals &goal, Cell curr, Dir facing,
         std::vector<Move> &current, std::unordered_set<Cell, CellHash> &visited,
         std::vector<std::vector<Move>> &solutions) {
  if (atGoal(curr, goal)) {
    solutions.push_back(current);
    solutions.back().push_back(Move::Stop);
    return;
  }

  visited.insert(curr);

  for (const Dir d : SCAN_ORDER) {
    if (!map.isOpen(curr, d))
      continue;
    const Cell adj = step(curr, d);
    if (visited.count(adj))
      continue;
    if (!map.isKnown(adj))
      continue;

    current.push_back(relativeMove(facing, d));
    dfs(map, goal, adj, d, current, visited, solutions);
    current.pop_back();
  }

  visited.erase(curr);
}

double distance(const std::vector<Move> &moves) {
  double cells = 0;
  for (const Move m : moves)
    if (m != Move::Stop)
      ++cells;
  return cells;
}

} // namespace

std::vector<Move> planFastRoute(const MazeMap &map, const Goals &goal,
                                Cell start, Dir facing) {
  std::vector<Move> current{};
  std::unordered_set<Cell, CellHash> visited{};
  std::vector<std::vector<Move>> solutions{};

  dfs(map, goal, start, facing, current, visited, solutions);

  double bestWeight = std::numeric_limits<double>::infinity();
  std::vector<Move> bestVec{};

  for (const auto &vec : solutions) {
    const double weight = distance(vec);
    if (weight < bestWeight) {
      bestWeight = weight;
      bestVec = vec;
    }
  }
  return bestVec;
}
