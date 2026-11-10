#pragma once
#include <cmath>

struct WorldCoord {
  double x = 0;
  double y = 0;
  double hypot() const { return std::sqrt(x * x + y * y); }
};

// theta = 0 is to the right, rotates counterclockwise. in radians.
struct Pose {
  WorldCoord xy{};
  double theta{};
};

struct KinematicState {
  double position = 0;
  double velocity = 0;
};

struct KinematicConstraints {
  double maxSpeed = 0;
  double maxAccel = 0;
};
