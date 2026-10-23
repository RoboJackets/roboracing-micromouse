#pragma once
#include <cmath>

// theta = 0 is to the right, rotates counterclockwise. in radians.
struct WorldCoord {
  double x = 0;
  double y = 0;
  double theta = 0;
};

WorldCoord operator-(const WorldCoord& lhs, const WorldCoord& rhs) {
  return WorldCoord{
    lhs.x - rhs.x,
    lhs.y - rhs.y,
    lhs.theta
  };
}

struct Translation2d {
  double x;
  double y;

  Translation2d(double x, double y) : x{x}, y{y} {}

  Translation2d operator-() const {
    return Translation2d{
      -x,
      -y
    };
  }

  Translation2d operator+=(const Translation2d& other) {
    this->x += other.x;
    this->y += other.y;
    return *this;
  }

  Translation2d operator-=(const Translation2d& other) {
    this->x -= other.x;
    this->y -= other.y;
    return *this;
  }

  double hypot() const { return std::sqrt(x * x + y * y); }
};

Translation2d operator+(const Translation2d& lhs, const Translation2d& rhs) {
  return Translation2d{
    lhs.x + rhs.x,
    lhs.y + rhs.y
  };
}

Translation2d operator-(const Translation2d& lhs, const Translation2d& rhs) {
  return lhs + -rhs;
}
