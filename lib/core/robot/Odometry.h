#pragma once
#include "Types.h"

struct Odometry {
  Translation2d w{};
  double theta = 0;
  double gyroOffset = 0;
  double gyroYaw = 0;
  bool seeded = false;

  double lastLeftPosition = 0;
  double lastRightPosition = 0;
  double leftPosition = 0;
  double rightPosition = 0;

  double filteredSpeedLeft = 0;
  double filteredSpeedRight = 0;
  double filteredRotationRate = 0;

  void update(double leftMeters, double rightMeters, double yaw, double dt);

  Translation2d pose() const { return w; }
  double heading() const { return theta; }
  void setPose(Translation2d c) { w = c; }
  void setGyroOffset(double offset) { gyroOffset = offset; }
  double rawGyroYaw() const { return gyroYaw; }

  double leftSpeed() const { return filteredSpeedLeft; }
  double rightSpeed() const { return filteredSpeedRight; }
  double rotationRate() const { return filteredRotationRate; }
};
