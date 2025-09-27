#include "include\FloodFillSolver.h"

#include <queue>
#include <string>

#include "../mms-cpp/API.h"

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

  std::queue<Coord> queue{};
  for (int i = 0; i < goal->count; ++i) {
    const int* g = goal->cells[i];
    state.dists[g[0]][g[1]] = 0;
    queue.push({g[1], g[0]});
  }

  while (!queue.empty()) {
    const Coord c = queue.front();
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

void updateWalls(MouseState& state) {
  unsigned char localWalls = 0;
  if (API::wallLeft()) localWalls |= LEFT;
  if (API::wallRight()) localWalls |= RIGHT;
  if (API::wallFront()) localWalls |= TOP;

  if (state.dir == LEFT) localWalls = LCIRC4(localWalls);
  if (state.dir == RIGHT) localWalls = RCIRC4(localWalls);
  if (state.dir == DOWN) localWalls = LCIRC4(LCIRC4(localWalls));

  state.walls[state.y][state.x] |= localWalls;

  if (state.x > 0 && (localWalls & LEFT)) {
    API::setWall(state.x, state.y, 'w');
    state.walls[state.y][state.x - 1] |= RIGHT;
  }
  if (state.x < N - 1 && (localWalls & RIGHT)) {
    API::setWall(state.x, state.y, 'e');
    state.walls[state.y][state.x + 1] |= LEFT;
  }
  if (state.y > 0 && (localWalls & DOWN)) {
    API::setWall(state.x, state.y, 's');
    state.walls[state.y - 1][state.x] |= TOP;
  }
  if (state.y < N - 1 && (localWalls & TOP)) {
    API::setWall(state.x, state.y, 'n');
    state.walls[state.y + 1][state.x] |= DOWN;
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

void traverse(MouseState& state, const Goals* goal) {
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

  unsigned char localBestDir = bestDir;
  if (state.dir == LEFT) localBestDir = RCIRC4(bestDir);
  if (state.dir == RIGHT) localBestDir = LCIRC4(bestDir);
  if (state.dir == DOWN) localBestDir = LCIRC4(LCIRC4(bestDir));

  switch (localBestDir) {
    case LEFT:
      API::turnLeft();
      break;
    case RIGHT:
      API::turnRight();
      break;
    case DOWN:
      API::turnLeft();
      API::turnLeft();
      break;
    default:
      break;
  }

  state.dir = bestDir;
  API::moveForward();

  switch (bestDir) {
    case TOP:
      ++state.y;
      break;
    case LEFT:
      --state.x;
      break;
    case RIGHT:
      ++state.x;
      break;
    case DOWN:
      --state.y;
      break;
    default:
      break;
  }

  state.explored[state.y][state.x] = true;
  API::setColor(state.x, state.y, 'B');
}
}  // namespace

void FloodFillSolver::run(MouseState& state, const Goals* goal) {
  updateWalls(state);
  floodFill(state, goal);
  logCells(state);
  traverse(state, goal);
}

bool FloodFillSolver::end(const MouseState& state, const Goals* goal) {
  return atGoal(state, goal);
}
