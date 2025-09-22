#include <iostream>
#include <queue>
#include <string>

#include "../mms-cpp/API.h"
#include "Mouse.h"

void log(const std::string& text) { std::cerr << text << std::endl; }
int dists[16][16];
unsigned char walls[16][16];  // 1000: top, 0100: right, 0010: down, 0001: left
MouseState state{};

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
  }
  return 'x';
}

void logCells() {
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      API::setText(x, y, std::to_string(dists[y][x]));
    }
  }
}

void floodFill() {
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      dists[y][x] = 300;
    }
  }
  std::queue<Coord> queue{};
  for (int i = 0; i < state.currentGoals.count; i++) {
    const int* g = state.currentGoals.cells[i];
    dists[g[0]][g[1]] = 0;
    queue.push({g[1], g[0]});
  }
  while (!queue.empty()) {
    Coord c = queue.front();
    queue.pop();

    unsigned char wall = walls[c.y][c.x];
    int dist = dists[c.y][c.x];
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
  if (API::wallLeft()) {
    localWalls |= LEFT;
  }
  if (API::wallRight()) {
    localWalls |= RIGHT;
  }
  if (API::wallFront()) {
    localWalls |= TOP;
  }

  if (state.dir == LEFT) {
    localWalls = LCIRC4(localWalls);
  }
  if (state.dir == RIGHT) {
    localWalls = RCIRC4(localWalls);
  }
  if (state.dir == DOWN) {
    localWalls = LCIRC4(LCIRC4(localWalls));
  }
  walls[state.y][state.x] |= localWalls;
  if (state.x > 0 && (localWalls & LEFT)) {
    API::setWall(state.x, state.y, 'w');
    walls[state.y][state.x - 1] |= RIGHT;
  }
  if (state.x < 15 && (localWalls & RIGHT)) {
    API::setWall(state.x, state.y, 'e');
    walls[state.y][state.x + 1] |= LEFT;
  }
  if (state.y > 0 && (localWalls & DOWN)) {
    API::setWall(state.x, state.y, 's');
    walls[state.y - 1][state.x] |= TOP;
  }
  if (state.y < 15 && (localWalls & TOP)) {
    API::setWall(state.x, state.y, 'n');
    walls[state.y + 1][state.x] |= DOWN;
  }
}

void traverse() {
  updateWalls();
  floodFill();
  int currentDist = dists[state.y][state.x];
  int best = 300;
  unsigned char bestDir = TOP;
  if (!(walls[state.y][state.x] & TOP) && dists[state.y + 1][state.x] < best) {
    best = dists[state.y + 1][state.x];
    bestDir = TOP;
  }
  if (!(walls[state.y][state.x] & LEFT) && dists[state.y][state.x - 1] < best) {
    best = dists[state.y][state.x - 1];
    bestDir = LEFT;
  }
  if (!(walls[state.y][state.x] & DOWN) && dists[state.y - 1][state.x] < best) {
    best = dists[state.y - 1][state.x];
    bestDir = DOWN;
  }
  if (!(walls[state.y][state.x] & RIGHT) &&
      dists[state.y][state.x + 1] < best) {
    best = dists[state.y][state.x + 1];
    bestDir = RIGHT;
  }

  unsigned char localBestDir = bestDir;
  if (state.dir == LEFT) {
    localBestDir = RCIRC4(bestDir);
  }
  if (state.dir == RIGHT) {
    localBestDir = LCIRC4(bestDir);
  }
  if (state.dir == DOWN) {
    localBestDir = LCIRC4(LCIRC4(bestDir));
  }
  std::cerr << "State : " << convertDir(state.dir) << ' ' << state.x << ','
            << state.y << std::endl;
  std::cerr << "Directions: " << convertDir(bestDir) << ' '
            << convertDir(localBestDir) << std::endl;
  std::cerr << std::endl;

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
  }

  switch (bestDir) {
    case TOP:
      state.y++;
      break;
    case LEFT:
      state.x--;
      break;
    case RIGHT:
      state.x++;
      break;
    case DOWN:
      state.y--;
      break;
  }
  state.dir = bestDir;
  API::moveForward();
}

bool atGoal() {
  for (int i = 0; i < state.currentGoals.count; ++i) {
    const int* g = state.currentGoals.cells[i];
    if (g[0] == state.y && g[1] == state.x) return true;
  }
  return false;
}

int main(int argc, char* argv[]) {
  // init border:
  for (int i = 0; i < 16; i++) {
    walls[i][0] |= LEFT;
    walls[0][i] |= DOWN;
    walls[i][15] |= RIGHT;
    walls[15][i] |= TOP;
    API::setWall(0, i, 'w');
    API::setWall(i, 0, 's');
    API::setWall(15, i, 'e');
    API::setWall(i, 15, 'n');
  }
  log("Running...");
  API::setColor(0, 0, 'G');
  API::setText(0, 0, "start");
  while (true) {
    while (!atGoal()) {
      traverse();
      logCells();
    }
    state.toggleGoalType();
  }
}