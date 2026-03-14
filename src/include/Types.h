#pragma once
#include <cmath>
struct GridCoord {
  int x = 0;
  int y = 0;
  unsigned char dir;
};
// theta = 0 is to the right, rotates counterclockwise. in radians.
struct WorldCoord {
  double x = 0;
  double y = 0;
  double theta = 0;
  double hypot() { return std::sqrt(x * x + y * y); }
  struct WorldCoord gridRelativeCoords() {
    double rx = std::fmod(x, CELL_SIZE_METERS);
    double ry = std::fmod(y, CELL_SIZE_METERS);
    if (rx < 0) rx += CELL_SIZE_METERS;
    if (ry < 0) ry += CELL_SIZE_METERS;
    return WorldCoord{rx, ry, theta};
  }
};

constexpr unsigned char TOP{0b1000};
constexpr unsigned char RIGHT{0b0100};
constexpr unsigned char DOWN{0b0010};
constexpr unsigned char LEFT{0b0001};