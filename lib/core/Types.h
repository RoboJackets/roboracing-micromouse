#pragma once
#include <cmath>

struct WheelSpeeds {
  double left;
  double right;
};

struct Translation2d {
  double x = 0;
  double y = 0;

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

  double dot(const Translation2d& other) const { return x * other.x + y * other.y; }
  double hypot() const { return std::sqrt(x * x + y * y); }
  double angleDegrees() const { return std::atan2(y, x) * 180.0 / M_PI; }
  Translation2d normalized() const {
    double n = hypot();
    return n > 0 ? Translation2d{x / n, y / n} : Translation2d{};
  }
};

inline Translation2d operator+(const Translation2d& lhs, const Translation2d& rhs) {
  return Translation2d{
    lhs.x + rhs.x,
    lhs.y + rhs.y
  };
}

inline Translation2d operator-(const Translation2d& lhs, const Translation2d& rhs) {
  return lhs + -rhs;
}

inline Translation2d operator*(const Translation2d& lhs, double scalar) {
  return Translation2d{
    lhs.x * scalar,
    lhs.y * scalar
  };
}

inline Translation2d operator*(double scalar, const Translation2d& rhs) {
  return rhs * scalar;
}

inline double operator*(const Translation2d& lhs, const Translation2d& rhs) {
  return lhs.dot(rhs);
}

// theta = 0 is to the right, rotates counterclockwise. in radians.
struct Pose2d {
  Translation2d xy;
  double theta = 0;
};
