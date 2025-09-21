#include <iostream>
#include <queue>
#include <string>

#include "../mms-cpp/API.h"

struct Coord {
  int x;
  int y;
};
void log(const std::string &text) { std::cerr << text << std::endl; }
int dists[16][16];
int goals[][2] = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
unsigned char walls[16][16];  // 1000: left, 0100: down, 0010: up, 0001: right

int main(int argc, char *argv[]) {
  int x = 0, y = 0, dir = 0;
  for (int i = 0; i < 16; i++) {
    walls[i][0] |= 0b1000;
    walls[0][i] |= 0b0100;
    walls[i][15] |= 0b0001;
    walls[15][i] |= 0b0010;
  }
  log("Running...");
  API::setColor(0, 0, 'G');
  API::setText(0, 0, "start");
  while (!atGoal(x, y)) {
  }
}

void floodFill() {
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      dists[y][x] = 300;
    }
  }
  std::queue<Coord> queue{};
  for (auto &g : goals) {
    dists[g[0]][g[1]] = 0;
    queue.push({g[1], g[0]});
  }
  while (!queue.empty()) {
    Coord c = queue.front();
    queue.pop();

    unsigned char wall = walls[c.y][c.x];
    int dist = dists[c.y][c.x];
    // left
    if (!(wall & 0b1000) && dist + 1 < dists[c.y][c.x - 1]) {
      dists[c.y][c.x - 1] = dist + 1;
      queue.push({c.x - 1, c.y});
    }
    // right
    if (!(wall & 0b0001) && dist + 1 < dists[c.y][c.x + 1]) {
      dists[c.y][c.x + 1] = dist + 1;
      queue.push({c.x + 1, c.y});
    }
    // down
    if (!(wall & 0b0100) && dist + 1 < dists[c.y - 1][c.x]) {
      dists[c.y - 1][c.x] = dist + 1;
      queue.push({c.x, c.y - 1});
    }
    // up
    if (!(wall & 0b0010) && dist + 1 < dists[c.y + 1][c.x]) {
      dists[c.y + 1][c.x] = dist + 1;
      queue.push({c.x, c.y + 1});
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