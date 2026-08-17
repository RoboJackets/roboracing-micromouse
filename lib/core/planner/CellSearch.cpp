#include "CellSearch.h"

#include <sstream>
#include <string>
#include <unordered_set>

#include "CommandGenerator.h"

namespace {

struct CellHash {
  std::size_t operator()(const Cell &c) const { return c.x * N + c.y; }
};

void dfs(const MazeMap &map, const Goals &goal, Cell curr,
         std::vector<Cell> &current, std::unordered_set<Cell, CellHash> &visited,
         std::vector<std::vector<Cell>> &solutions) {
  if (atGoal(curr, goal)) {
    solutions.push_back(current);
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

    current.push_back(adj);
    dfs(map, goal, adj, current, visited, solutions);
    current.pop_back();
  }

  visited.erase(curr);
}

std::string path_to_instruct(const std::vector<Cell> &path, Dir facing) {
  const Cell heading = step(Cell{0, 0}, facing);
  int curr_dx = heading.x;
  int curr_dy = heading.y;

  std::stringstream ss;
  ss << 'X';
  for (size_t i{1}; i < path.size(); ++i) {
    auto prev = path[i - 1];
    auto curr = path[i];

    int dx = curr.x - prev.x;
    int dy = curr.y - prev.y;

    if (dx == curr_dx && dy == curr_dy) { // straight ahead
      ss << 'F';
    } else if (dx == curr_dy && dy == -curr_dx) { // right turn
      ss << 'R';
    } else if (dx == -curr_dy && dy == curr_dx) { // left turn
      ss << 'L';
    } else if (dx == -curr_dx && dy == -curr_dy) { // 180 degree turn
      ss << 'B';
    }

    curr_dx = dx;
    curr_dy = dy;
  }

  ss << 'S';

  return ss.str();
}

} // namespace

std::vector<unsigned char> planFastRoute(const MazeMap &map, const Goals &goal,
                                         Cell start, Dir facing) {
  std::vector<Cell> temp{};
  std::unordered_set<Cell, CellHash> visited{};
  std::vector<std::vector<Cell>> solutions{};

  temp.push_back(start);
  dfs(map, goal, start, temp, visited, solutions);

  double bestWeight = 9999999999999.0;
  std::vector<unsigned char> bestVec{};

  for (const auto &vec : solutions) {
    std::string s = path_to_instruct(vec, facing);
    std::vector<unsigned char> v = parse(s, false);
    double w = computeWeight(v);
    if (w < bestWeight) {
      bestWeight = w;
      bestVec = std::move(v);
    }
  }
  return bestVec;
}
