#include "CellSelection.h"

void floodFill(MouseState &state, const Goals *goal, int (&dists)[N][N]) {
  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      dists[y][x] = INF;
    }
  }

  std::queue<GridCoord> queue{};
  for (int i = 0; i < goal->count; ++i) {
    const int *g = goal->cells[i];
    dists[g[0]][g[1]] = 0;
    queue.push({g[1], g[0]});
  }

  while (!queue.empty()) {
    const GridCoord c = queue.front();
    queue.pop();

    const unsigned char wall = state.walls[c.y][c.x];
    const int dist = dists[c.y][c.x];

    // left
    if (c.x - 1 >= 0 && !(wall & LEFT) && state.explored[c.y][c.x - 1] &&
        dist + 1 < dists[c.y][c.x - 1]) {
      dists[c.y][c.x - 1] = dist + 1;
      queue.push({c.x - 1, c.y});
    }
    // right
    if (c.x + 1 < N && !(wall & RIGHT) && state.explored[c.y][c.x + 1] &&
        dist + 1 < dists[c.y][c.x + 1]) {
      dists[c.y][c.x + 1] = dist + 1;
      queue.push({c.x + 1, c.y});
    }
    // down
    if (c.y - 1 >= 0 && !(wall & DOWN) && state.explored[c.y - 1][c.x] &&
        dist + 1 < dists[c.y - 1][c.x]) {
      dists[c.y - 1][c.x] = dist + 1;
      queue.push({c.x, c.y - 1});
    }
    // up
    if (c.y + 1 < N && !(wall & TOP) && state.explored[c.y + 1][c.x] &&
        dist + 1 < dists[c.y + 1][c.x]) {
      dists[c.y + 1][c.x] = dist + 1;
      queue.push({c.x, c.y + 1});
    }
  }
}
namespace CellSelection {
Path pathBFS(MouseState &state, const Goals *goal) {
  int dists[N][N];
  floodFill(state, goal, dists);
  Path finalPath{{GridCoord{}}};
  while (true) {
    GridCoord current = finalPath.steps.at(finalPath.steps.size() - 1);
    int best = INF;
    unsigned char bestDir = TOP;
    unsigned char wall = state.walls[current.y][current.x];
    int dist = dists[current.y][current.x];
    if (current.y + 1 < N && !(wall & TOP) &&
        state.explored[current.y + 1][current.x] &&
        dists[current.y + 1][current.x] < best &&
        dists[current.y + 1][current.x] < dist) {
      best = dists[current.y + 1][current.x];
      bestDir = TOP;
    }
    if (current.x - 1 >= 0 && !(wall & LEFT) &&
        state.explored[current.y][current.x - 1] &&
        dists[current.y][current.x - 1] < best &&
        dists[current.y][current.x - 1] < dist) {
      best = dists[current.y][current.x - 1];
      bestDir = LEFT;
    }
    if (current.y - 1 >= 0 && !(wall & DOWN) &&
        state.explored[current.y - 1][current.x] &&
        dists[current.y - 1][current.x] < best &&
        dists[current.y - 1][current.x] < dist) {
      best = dists[current.y - 1][current.x];
      bestDir = DOWN;
    }
    if (current.x + 1 < N && !(wall & RIGHT) &&
        state.explored[current.y][current.x + 1] &&
        dists[current.y][current.x + 1] < best &&
        dists[current.y][current.x + 1] < dist) {
      best = dists[current.y][current.x + 1];
      bestDir = RIGHT;
    }
    GridCoord nextDirection = dirToVector(bestDir);
    GridCoord nextCur =
        GridCoord{current.x + nextDirection.x, current.y + nextDirection.y};
    finalPath.steps.push_back(nextCur);
    if (dists[nextCur.y][nextCur.x] == 0) {
      break;
    }
  }
  for (int i = 0; i < finalPath.steps.size(); i++) {
    GridCoord step = finalPath.steps[i];
  }
  return finalPath;
}

struct pair_hash {
  inline std::size_t operator()(const std::pair<int, int> &v) const {
    return v.first * (N) + v.second;
  }
};

using IntPair = std::pair<int, int>;

inline bool blocked(const MouseState &s, int x, int y, int nx, int ny) {
  if (nx == x && ny == y + 1)
    return s.walls[y][x] & TOP;
  if (nx == x && ny == y - 1)
    return s.walls[y][x] & DOWN;
  if (nx == x + 1 && ny == y)
    return s.walls[y][x] & RIGHT;
  if (nx == x - 1 && ny == y)
    return s.walls[y][x] & LEFT;
  return true;
}

inline bool at_goal(IntPair pos, const Goals *goal) {
  for (int i = 0; i < goal->count; ++i) {
    if (pos.first == goal->cells[i][1] && pos.second == goal->cells[i][0])
      return true;
  }
  return false;
}

int dirs[][2] = {
    {0, 1},
    {1, 0},
    {0, -1},
    {-1, 0},
};

void dfs(const MouseState &state, const Goals *goal, IntPair curr,
         std::vector<IntPair> &current,
         std::unordered_set<IntPair, pair_hash> &visited,
         std::vector<std::vector<IntPair>> &solutions) {
  if (at_goal(curr, goal)) {
    solutions.push_back(current);
    return;
  }

  visited.insert(curr);

  for (auto &dir : dirs) {
    int adj_x = curr.first + dir[0];
    int adj_y = curr.second + dir[1];

    if (adj_x < 0 || adj_x >= N || adj_y < 0 || adj_y >= N)
      continue;
    IntPair adj(adj_x, adj_y);
    if (visited.count(adj))
      continue;
    if (!state.explored[adj_y][adj_x])
      continue;
    if (blocked(state, curr.first, curr.second, adj_x, adj_y))
      continue;

    current.push_back(adj);
    dfs(state, goal, adj, current, visited, solutions);
    current.pop_back();
  }

  visited.erase(curr);
}

std::string path_to_instruct(const std::vector<IntPair> &path) {
  int curr_dx = 0;
  int curr_dy = 1;

  std::stringstream ss;
  ss << 'X';
  for (int i{1}; i < path.size(); ++i) {
    auto prev = path[i - 1];
    auto curr = path[i];

    int dx = curr.first - prev.first;
    int dy = curr.second - prev.second;

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
std::vector<unsigned char> cmds;
void search_all(const MouseState &state, const Goals *goal) {
  std::vector<IntPair> temp{};
  std::unordered_set<IntPair, pair_hash> visited{};
  std::vector<std::vector<IntPair>> solutions{};
  Serial.println(state.dists[0][0]);

  IntPair start{0, 0};
  temp.push_back(start);
  dfs(state, goal, start, temp, visited, solutions);

  double bestWeight = 9999999999999.0;
  std::vector<unsigned char> bestVec{};

  for (const auto &vec : solutions) {
    std::string s = path_to_instruct(vec);
    // std::cerr << s << std::endl;
    std::vector<unsigned char> v = std::move(parse(s, false));
    double w = computeWeight(v);
    Serial.println(w);
    if (w < bestWeight) {
      bestWeight = w;
      bestVec = std::move(v);
    }
  }
  cmds = bestVec;
}
std::vector<unsigned char> getCmds() { return cmds; }
} // namespace CellSelection