#pragma once
#include <iostream>
#include <string>
#include <vector>

#include "Types.h"
#define RCIRC4(x) (((x) & 1) ? (0b1000 | (x) >> 1) : (x) >> 1)
#define LCIRC4(x) (((x) & 0b1000) ? (0b0001 | ((x) & ~0b1000) << 1) : (x) << 1)

inline void log(const std::string& text) { std::cerr << text << '\n'; }

constexpr int N = 16;
constexpr int INF = 300;

struct Path {
  std::vector<GridCoord> steps;
};

inline int dirToDist(unsigned char dir1, unsigned char dir2) {
  if (dir1 == dir2) return 0;
  unsigned char rot1 = dir1, rot2 = dir2;
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
  return (rot2 == LEFT || rot2 == RIGHT) ? 1 : 2;
}

inline char convertDir(unsigned char dir) {
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

inline GridCoord dirToVector(int dir) {
  switch (dir) {
    case 0:
      return {-1, 0};
    case 1:
      return {1, 0};
    case 2:
      return {0, -1};
    case 3:
      return {0, 1};
    default:
      return {0, 0};
  }
}
inline unsigned char dirToChar(int dir) {
  switch (dir) {
    case 0:
      return LEFT;
    case 1:
      return RIGHT;
    case 2:
      return DOWN;
    case 3:
      return TOP;
    default:
      return TOP;
  }
}
inline unsigned char vectorToDir(const GridCoord& vec) {
  if (vec.x == -1 && vec.y == 0) return LEFT;
  if (vec.x == 1 && vec.y == 0) return RIGHT;
  if (vec.x == 0 && vec.y == -1) return DOWN;
  if (vec.x == 0 && vec.y == 1) return TOP;
  return 0;
}

inline GridCoord dirToVector(unsigned char dir) {
  switch (dir) {
    case LEFT:
      return {-1, 0};
    case RIGHT:
      return {1, 0};
    case DOWN:
      return {0, -1};
    case TOP:
      return {0, 1};
    default:
      return {0, 0};
  }
}