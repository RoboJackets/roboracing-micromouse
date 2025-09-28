#include "../include/FloodFillSolver.h"

#include <queue>
#include <string>

#include "../../mms-cpp/API.h"
#include "../Actions/MoveAction.h"

namespace {
void logCells(MouseState& state) {
  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      API::setText(x, y, std::to_string(state.dists[y][x]));
    }
  }
}

void turningPenalty(MouseState& state, const Goals* goal) {
  if (!(state.walls[state.y][state.x] & TOP)) {
    state.dists[state.y + 1][state.x] +=
        goal->turnPenalty * dirToDist(state.dir, TOP);
  }
  if (!(state.walls[state.y][state.x] & LEFT)) {
    state.dists[state.y][state.x - 1] +=
        goal->turnPenalty * dirToDist(state.dir, LEFT);
  }
  if (!(state.walls[state.y][state.x] & DOWN)) {
    state.dists[state.y - 1][state.x] +=
        goal->turnPenalty * dirToDist(state.dir, DOWN);
  }
  if (!(state.walls[state.y][state.x] & RIGHT)) {
    state.dists[state.y][state.x + 1] +=
        goal->turnPenalty * dirToDist(state.dir, RIGHT);
  }
}

void floodFill(MouseState& state, const Goals* goal) {
  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      state.dists[y][x] = INF;
    }
  }

  std::queue<GridCoord> queue{};
  for (int i = 0; i < goal->count; ++i) {
    const int* g = goal->cells[i];
    state.dists[g[0]][g[1]] = 0;
    queue.push({g[1], g[0]});
  }

  while (!queue.empty()) {
    const GridCoord c = queue.front();
    queue.pop();

    const unsigned char wall = state.walls[c.y][c.x];
    const int dist = state.dists[c.y][c.x];

    // left
    if (!(wall & LEFT) && dist + 1 < state.dists[c.y][c.x - 1]) {
      state.dists[c.y][c.x - 1] = dist + 1;
      queue.push({c.x - 1, c.y});
    }
    // right
    if (!(wall & RIGHT) && dist + 1 < state.dists[c.y][c.x + 1]) {
      state.dists[c.y][c.x + 1] = dist + 1;
      queue.push({c.x + 1, c.y});
    }
    // down
    if (!(wall & DOWN) && dist + 1 < state.dists[c.y - 1][c.x]) {
      state.dists[c.y - 1][c.x] = dist + 1;
      queue.push({c.x, c.y - 1});
    }
    // up
    if (!(wall & TOP) && dist + 1 < state.dists[c.y + 1][c.x]) {
      state.dists[c.y + 1][c.x] = dist + 1;
      queue.push({c.x, c.y + 1});
    }
  }
}

void applyTiebreaker(MouseState& state, const Goals* goal) {
  turningPenalty(state, goal);
  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      if (!state.explored[y][x]) {
        state.dists[y][x] -= goal->explorationWeight;
      }
    }
  }
}

unsigned char traverse(MouseState& state, const Goals* goal) {
  auto fillNeighborCosts = [&](int out[4]) {
    out[0] = out[1] = out[2] = out[3] = INF + 200;
    const unsigned char here = state.walls[state.y][state.x];
    if (!(here & TOP)) out[0] = state.dists[state.y + 1][state.x];
    if (!(here & LEFT)) out[1] = state.dists[state.y][state.x - 1];
    if (!(here & DOWN)) out[2] = state.dists[state.y - 1][state.x];
    if (!(here & RIGHT)) out[3] = state.dists[state.y][state.x + 1];
  };

  int bestDirArray[4];
  fillNeighborCosts(bestDirArray);

  int best = INF + 100;
  bool tie = false;
  int bestDirID = -1;

  for (int i = 0; i < 4; ++i) {
    if (bestDirArray[i] == best) tie = true;
    if (bestDirArray[i] < best) {
      best = bestDirArray[i];
      bestDirID = i;
      tie = false;
    }
  }

  if (tie) {
    applyTiebreaker(state, goal);
    logCells(state);
    fillNeighborCosts(bestDirArray);
    best = INF + 100;
    bestDirID = -1;
    for (int i = 0; i < 4; ++i) {
      if (bestDirArray[i] < best) {
        best = bestDirArray[i];
        bestDirID = i;
      }
    }
  }

  unsigned char bestDir = 0;
  switch (bestDirID) {
    case 0:
      bestDir = TOP;
      break;
    case 1:
      bestDir = LEFT;
      break;
    case 2:
      bestDir = DOWN;
      break;
    case 3:
      bestDir = RIGHT;
      break;
    default:
      bestDir = state.dir;
      break;
  }
  return bestDir;
}
}  // namespace

Action FloodFillSolver::run(MouseState& state, const Goals* goal) {
  floodFill(state, goal);
  logCells(state);
  unsigned char dir = traverse(state, goal);
  GridCoord v = dirToVector(dir);
  IdealState idealState = IdealState{state.x + v.x, state.y + v.y};
  return MoveAction(idealState);
}

bool FloodFillSolver::end(MouseState& state, const Goals* goal) {
  log("wow!");
  return atGoal(state, goal);
}
