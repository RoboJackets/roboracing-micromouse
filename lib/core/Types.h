#pragma once
#include <cmath>

// theta = 0 is to the right, rotates counterclockwise. in radians.
struct WorldCoord {
  double x = 0;
  double y = 0;

  double dot(const WorldCoord& other) const { return x * other.x + y * other.y; }
  double hypot() const { return std::sqrt(x * x + y * y); }
  double angleDegrees() const { return std::atan2(y, x) * 180.0 / M_PI; }
  WorldCoord normalized() const {
    double n = hypot();
    return n > 0 ? WorldCoord{x / n, y / n} : WorldCoord{};
  }
};

WorldCoord operator-(const WorldCoord& lhs, const WorldCoord& rhs) {
  return WorldCoord{
    lhs.x - rhs.x,
    lhs.y - rhs.y
  };
}

inline WorldCoord operator*(const WorldCoord& lhs, double scalar) {
  return WorldCoord{
    lhs.x * scalar,
    lhs.y * scalar
  };
}

inline WorldCoord operator*(double scalar, const WorldCoord& rhs) {
  return rhs * scalar;
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

struct Pose2d {
  Translation2d xy;
  double theta;
};

double operator*(const Translation2d& lhs, const Translation2d& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}