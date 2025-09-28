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
Path selectPath(MouseState& state, const Goals* goal) {
  int dists[N][N];
  floodFill(state, goal, dists);
  Path finalPath{{Coord{}}};
  while (true) {
    Coord current = finalPath.steps.at(finalPath.steps.size() - 1);
    int best = INF;
    unsigned char bestDir = TOP;
    unsigned char wall = state.walls[current.y][current.x];
    int dist = state.dists[current.y][current.x];
    if (!(wall & TOP) && state.explored[current.y + 1][current.x] &&
        state.dists[current.y + 1][current.x] < best) {
      best = state.dists[current.y + 1][current.x];
      bestDir = TOP;
    }
    if (!(wall & LEFT) && state.explored[current.y][current.x - 1] &&
        state.dists[current.y][current.x - 1] < best) {
      best = state.dists[current.y][current.x - 1];
      bestDir = LEFT;
    }
    if (!(wall & DOWN) && state.explored[current.y - 1][current.x] &&
        state.dists[current.y - 1][current.x] < best) {
      best = state.dists[current.y - 1][current.x];
      bestDir = DOWN;
    }
    if (!(wall & RIGHT) && state.explored[current.y][current.x + 1] &&
        state.dists[current.y][current.x + 1] < best) {
      best = state.dists[current.y][current.x + 1];
      bestDir = RIGHT;
    }
    Coord nextDirection = dirToVector(bestDir);
    Coord nextCur =
        Coord{current.x + nextDirection.x, current.y + nextDirection.y};
    finalPath.steps.push_back(nextCur);
    if (state.dists[nextCur.y][nextCur.x] == 0){
      break;
    }
  }
  return finalPath;
}
}  // namespace CellSelection