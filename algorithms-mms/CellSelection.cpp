#include "include\CellSelection.h"

#include <queue>

#include "..\mms-cpp\API.h"
#include "include\Mouse.h"
void floodFill(MouseState& state, const Goals* goal, int (&dists)[N][N]) {
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

  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      API::setText(x, y, std::to_string(dists[y][x]));
      if (state.explored[y][x]) {
        API::setColor(x, y, 'G');
      } else {
        API::setColor(x, y, 'R');
      }
    }
  }
}
namespace CellSelection {
std::vector<Path> selectAllPaths(MouseState& state, const Goals* goal) {
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

      int adjDists[4] = {INF + 100, INF + 100, INF + 100, INF + 100};
      // left
      if (!(wall & LEFT) &&
          state.explored[currentPosition.y][currentPosition.x - 1] &&
          dists[currentPosition.y][currentPosition.x - 1] <
              dists[currentPosition.y][currentPosition.x]) {
        adjDists[0] = dists[currentPosition.y][currentPosition.x - 1];
      }
      // right
      if (!(wall & RIGHT) &&
          state.explored[currentPosition.y][currentPosition.x + 1] &&
          dists[currentPosition.y][currentPosition.x + 1] <
              dists[currentPosition.y][currentPosition.x]) {
        adjDists[1] = dists[currentPosition.y][currentPosition.x + 1];
      }
      // down
      if (!(wall & DOWN) &&
          state.explored[currentPosition.y - 1][currentPosition.x] &&
          dists[currentPosition.y - 1][currentPosition.x] <
              dists[currentPosition.y][currentPosition.x]) {
        adjDists[2] = dists[currentPosition.y - 1][currentPosition.x];
      }
      // up
      if (!(wall & TOP) &&
          state.explored[currentPosition.y + 1][currentPosition.x] &&
          dists[currentPosition.y + 1][currentPosition.x] <
              dists[currentPosition.y][currentPosition.x]) {
        adjDists[3] = dists[currentPosition.y + 1][currentPosition.x];
      }
      int best = 300;
      bool ties[4] = {false, false, false, false};

      for (int i = 0; i < 4; i++) {
        if (adjDists[i] == best) {
          ties[i] = true;
        }
        if (adjDists[i] < best) {
          for (int j = 0; j < 4; j++) ties[j] = false;
          best = adjDists[i];
          ties[i] = true;
        }
      }

      Path basePath = currentPath;  // snapshot before mutating

      bool element = false;
      for (int i = 0; i < 4; i++) {
        if (ties[i]) {
          Coord c = dirToVector(i);
          if (!element) {
            currentCells.push(
                {currentPosition.x + c.x, currentPosition.y + c.y});
            currentPath.steps.push_back(
                {currentPosition.x + c.x, currentPosition.y + c.y});
          } else {
            Path p =
                basePath;  // fork from snapshot, not the already-extended path
            p.steps.push_back(
                {currentPosition.x + c.x, currentPosition.y + c.y});
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
}  // namespace CellSelection