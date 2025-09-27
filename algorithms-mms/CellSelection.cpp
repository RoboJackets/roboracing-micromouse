#include <queue>

#include "include\Mouse.h"
static void floodFill(MouseState& state, Goals* goal, int (&dists)[N][N]) {
  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      dists[y][x] = INF;
    }
  }

  std::queue<Coord> queue{};
  for (int i = 0; i < goal->count; ++i) {
    const int* g = goal->cells[i];
    dists[g[0]][g[1]] = 0;
    queue.push({g[1], g[0]});
  }

  while (!queue.empty()) {
    const Coord c = queue.front();
    queue.pop();

    const unsigned char wall = state.walls[c.y][c.x];
    const int dist = dists[c.y][c.x];

    // left
    if (!(wall & LEFT) && state.explored[c.y][c.x - 1] &&
        dist + 1 < dists[c.y][c.x - 1]) {
      dists[c.y][c.x - 1] = dist + 1;
      queue.push({c.x - 1, c.y});
    }
    // right
    if (!(wall & RIGHT) && state.explored[c.y][c.x + 1] &&
        dist + 1 < dists[c.y][c.x + 1]) {
      dists[c.y][c.x + 1] = dist + 1;
      queue.push({c.x + 1, c.y});
    }
    // down
    if (!(wall & DOWN) && state.explored[c.y - 1][c.x] &&
        dist + 1 < dists[c.y - 1][c.x]) {
      dists[c.y - 1][c.x] = dist + 1;
      queue.push({c.x, c.y - 1});
    }
    // up
    if (!(wall & TOP) && state.explored[c.y + 1][c.x] &&
        dist + 1 < dists[c.y + 1][c.x]) {
      dists[c.y + 1][c.x] = dist + 1;
      queue.push({c.x, c.y + 1});
    }
  }
}

static std::vector<Path> selectAllPaths(MouseState& state, Goals* goal) {
  int dists[N][N];
  floodFill(state, goal, dists);
  std::queue<Path> pathQueue{};
  pathQueue.push(Path{{Coord{}}});
  std::vector<Path> paths;
  while (!pathQueue.empty()) {
    Path currentPath = pathQueue.front();
    pathQueue.pop();
    std::queue<Coord> currentCells{};
    currentCells.push(currentPath.steps[currentPath.steps.size() - 1]);
    while (!currentCells.empty()) {
      Coord currentPosition = currentCells.front();
      currentCells.pop();
      const unsigned char wall =
          state.walls[currentPosition.y][currentPosition.x];
      const int dist = dists[currentPosition.y][currentPosition.x];

      int dists[4] = {INF + 100, INF + 100, INF + 100, INF + 100};

      // left
      if (!(wall & LEFT) &&
          state.explored[currentPosition.y][currentPosition.x - 1]) {
        dists[0] = state.dists[currentPosition.y][currentPosition.x - 1];
      }
      // right
      if (!(wall & RIGHT) &&
          state.explored[currentPosition.y][currentPosition.x + 1]) {
        dists[1] = state.dists[currentPosition.y][currentPosition.x + 1];
      }
      // down
      if (!(wall & DOWN) &&
          state.explored[currentPosition.y - 1][currentPosition.x]) {
        dists[2] = state.dists[currentPosition.y - 1][currentPosition.x];
      }
      // up
      if (!(wall & TOP) &&
          state.explored[currentPosition.y + 1][currentPosition.x]) {
        dists[3] = state.dists[currentPosition.y + 1][currentPosition.x];
      }
      int best = 300;
      bool ties[4];
      for (int i = 0; i < 4; i++) {
        if (dists[i] == best) {
          ties[i] = true;
        }
        if (dists[i] < best) {
          for (int j = 0; j < 4; j++) {
            ties[j] = false;
          }
          ties[i] = true;
        }
      }
      bool element = false;
      for (int i = 0; i < 4; i++) {
        if (ties[i]) {
          if (!element) {
            Coord c = dirToVector(i);
            currentCells.push(
                Coord{currentPosition.x + c.x, currentPosition.y + c.y});
            currentPath.steps.push_back(
                Coord{currentPosition.x + c.x, currentPosition.y + c.y});
          } else {
            Coord c = dirToVector(i);
            Path p = Path{{currentPath.steps}};
            p.steps.push_back(
                Coord{currentPosition.x + c.x, currentPosition.y + c.y});
            pathQueue.push(p);
          }
          element = true;
        }
      }
    }
    paths.push_back(currentPath);
  }
  return paths;
}