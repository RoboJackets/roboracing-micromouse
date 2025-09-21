#include <iostream>
#include <string>

#include "../mms-cpp/API.h"
void log(const std::string &text) { std::cerr << text << std::endl; }
int dist[16][16];
int goals[][2] = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
bool walls[16][16][4];  // up, right, down, left

int main(int argc, char *argv[]) {
  int x = 0, y = 0, dir = 0;
  init();
  log("Running...");
  API::setColor(0, 0, 'G');
  API::setText(0, 0, "start");
  while (!atGoal(x, y)) {
    if (API::wallFront()) walls[x][y][dir] = true;
    if (API::wallLeft()) walls[x][y][(dir + 3) % 4] = true;
    if (API::wallRight()) walls[x][y][(dir + 1) % 4] = true;
  }
}
void init() {
  for (int y = 0; y < 16; y++) {
    for (int x = 0; x < 16; x++) {
      int best = 1e9;
      for (auto &g : goals) {
        int d = abs(x - g[0]) + abs(y - g[1]);
        if (d < best) best = d;
      }
      dist[y][x] = best;
    }
  }
}
bool atGoal(int x, int y) {
  for (auto &g : goals) {
    if (g[0] == x && g[1] == y) {
      return true;
    }
  }
  return false;
}