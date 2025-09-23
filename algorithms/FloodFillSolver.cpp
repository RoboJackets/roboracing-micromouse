#include <iostream>
#include <queue>
#include <string>

#include "../mms-cpp/API.h"
#include "Mouse.h"

void log(const std::string& text) { std::cerr << text << '\n'; }

constexpr int N = 16;
constexpr int INF = 300;

int dists[N][N];
bool explored[N][N];
unsigned char walls[N][N];  // 1000: top, 0100: right, 0010: down, 0001: left
MouseState state{};

bool atGoal() {
  for (int i = 0; i < state.currentGoals.count; ++i) {
    const int* g = state.currentGoals.cells[i];
    if (g[0] == state.y && g[1] == state.x) return true;
  }
  return false;
}

char convertDir(unsigned char dir) {
  switch (dir) {
    case TOP:
      return 'n';
    case LEFT:
      return 'w';
    case RIGHT:
      return 'e';
    case DOWN:
      return 's';
    default:
      return 'x';
  }
}

void logCells() {
  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      API::setText(x, y, std::to_string(dists[y][x]));
    }
  }
}

int dirToDist(unsigned char dir1, unsigned char dir2) {
  if (dir1 == dir2) return 0;

  unsigned char rot1 = dir1;
  unsigned char rot2 = dir2;

  if (dir1 == LEFT) {
    rot1 = RCIRC4(dir1);
    rot2 = RCIRC4(dir2);
  }
  if (dir1 == RIGHT) {
    rot1 = LCIRC4(dir1);
    rot2 = LCIRC4(dir2);
  }
  if (dir1 == DOWN) {
    rot1 = LCIRC4(LCIRC4(dir1));
    rot2 = LCIRC4(LCIRC4(dir2));
  }

  if (rot2 == LEFT || rot2 == RIGHT) return 1;
  return 2;
}

void turningPenalty() {
  if (!(walls[state.y][state.x] & TOP)) {
    dists[state.y + 1][state.x] +=
        state.currentGoals.turnPenalty * dirToDist(state.dir, TOP);
  }
  if (!(walls[state.y][state.x] & LEFT)) {
    dists[state.y][state.x - 1] +=
        state.currentGoals.turnPenalty * dirToDist(state.dir, LEFT);
  }
  if (!(walls[state.y][state.x] & DOWN)) {
    dists[state.y - 1][state.x] +=
        state.currentGoals.turnPenalty * dirToDist(state.dir, DOWN);
  }
  if (!(walls[state.y][state.x] & RIGHT)) {
    dists[state.y][state.x + 1] +=
        state.currentGoals.turnPenalty * dirToDist(state.dir, RIGHT);
  }
}

void floodFill() {
  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      dists[y][x] = INF;
    }
  }

  std::queue<Coord> queue{};
  for (int i = 0; i < state.currentGoals.count; ++i) {
    const int* g = state.currentGoals.cells[i];
    dists[g[0]][g[1]] = 0;
    queue.push({g[1], g[0]});
  }

  while (!queue.empty()) {
    const Coord c = queue.front();
    queue.pop();

    const unsigned char wall = walls[c.y][c.x];
    const int dist = dists[c.y][c.x];

    // left
    if (!(wall & LEFT) && dist + 1 < dists[c.y][c.x - 1]) {
      dists[c.y][c.x - 1] = dist + 1;
      queue.push({c.x - 1, c.y});
    }
    // right
    if (!(wall & RIGHT) && dist + 1 < dists[c.y][c.x + 1]) {
      dists[c.y][c.x + 1] = dist + 1;
      queue.push({c.x + 1, c.y});
    }
    // down
    if (!(wall & DOWN) && dist + 1 < dists[c.y - 1][c.x]) {
      dists[c.y - 1][c.x] = dist + 1;
      queue.push({c.x, c.y - 1});
    }
    // up
    if (!(wall & TOP) && dist + 1 < dists[c.y + 1][c.x]) {
      dists[c.y + 1][c.x] = dist + 1;
      queue.push({c.x, c.y + 1});
    }
  }
}

void updateWalls() {
  unsigned char localWalls = 0;
  if (API::wallLeft()) localWalls |= LEFT;
  if (API::wallRight()) localWalls |= RIGHT;
  if (API::wallFront()) localWalls |= TOP;

  if (state.dir == LEFT) localWalls = LCIRC4(localWalls);
  if (state.dir == RIGHT) localWalls = RCIRC4(localWalls);
  if (state.dir == DOWN) localWalls = LCIRC4(LCIRC4(localWalls));

  walls[state.y][state.x] |= localWalls;

  if (state.x > 0 && (localWalls & LEFT)) {
    API::setWall(state.x, state.y, 'w');
    walls[state.y][state.x - 1] |= RIGHT;
  }
  if (state.x < N - 1 && (localWalls & RIGHT)) {
    API::setWall(state.x, state.y, 'e');
    walls[state.y][state.x + 1] |= LEFT;
  }
  if (state.y > 0 && (localWalls & DOWN)) {
    API::setWall(state.x, state.y, 's');
    walls[state.y - 1][state.x] |= TOP;
  }
  if (state.y < N - 1 && (localWalls & TOP)) {
    API::setWall(state.x, state.y, 'n');
    walls[state.y + 1][state.x] |= DOWN;
  }
}

void applyTiebreaker() {
  turningPenalty();
  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      if (!explored[y][x]) {
        dists[y][x] -= state.currentGoals.explorationWeight;
      }
    }
  }
}

void traverse() {
  auto fillNeighborCosts = [&](int out[4]) {
    out[0] = out[1] = out[2] = out[3] = INF + 200;
    const unsigned char here = walls[state.y][state.x];
    if (!(here & TOP)) out[0] = dists[state.y + 1][state.x];
    if (!(here & LEFT)) out[1] = dists[state.y][state.x - 1];
    if (!(here & DOWN)) out[2] = dists[state.y - 1][state.x];
    if (!(here & RIGHT)) out[3] = dists[state.y][state.x + 1];
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
    applyTiebreaker();
    logCells();
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

  explored[state.y][state.x] = true;
  API::setColor(state.x, state.y, 'B');
}

int main() {
  explored[0][0] = true;
  for (int i = 0; i < CENTER_GOALS.count; ++i) {
    const int gx = CENTER_GOALS.cells[i][1];
    const int gy = CENTER_GOALS.cells[i][0];
    explored[gy][gx] = true;
  }

  for (int i = 0; i < N; ++i) {
    walls[i][0] |= LEFT;
    walls[0][i] |= DOWN;
    walls[i][N - 1] |= RIGHT;
    walls[N - 1][i] |= TOP;

    API::setWall(0, i, 'w');
    API::setWall(i, 0, 's');
    API::setWall(N - 1, i, 'e');
    API::setWall(i, N - 1, 'n');
  }

  log("Running...");
  API::setColor(0, 0, 'G');
  API::setText(0, 0, "start");

  int totalRuns = 0;
  while (true) {
    if (totalRuns >= 2) break;

    updateWalls();

    if (atGoal()) {
      log("goal!");
      state.toggleGoalType();
      floodFill();
      logCells();
      API::setColor(state.x, state.y, 'R');
      ++totalRuns;
      continue;
    }

    floodFill();
    logCells();
    traverse();
  }
}
