#include "Odometry.h"

#include <cmath>

#include "Tuning.h"

void Odometry::update(double leftMeters, double rightMeters, double yaw,
                      double dt) {
  if (!seeded) {
    seeded = true;
    gyroYaw = yaw;
    leftPosition = leftMeters;
    rightPosition = rightMeters;
  }

  const double prevYaw = gyroYaw;
  gyroYaw = yaw;
  const double rawRotationRate =
      std::remainder(gyroYaw - prevYaw, 2 * M_PI) / dt;
  filteredRotationRate +=
      ROTATION_FILTER_ALPHA * (rawRotationRate - filteredRotationRate);

  lastLeftPosition = leftPosition;
  lastRightPosition = rightPosition;
  leftPosition = leftMeters;
  rightPosition = rightMeters;

  const double rawLeft = (leftPosition - lastLeftPosition) / dt;
  const double rawRight = (rightPosition - lastRightPosition) / dt;
  filteredSpeedLeft += SPEED_FILTER_ALPHA * (rawLeft - filteredSpeedLeft);
  filteredSpeedRight += SPEED_FILTER_ALPHA * (rawRight - filteredSpeedRight);

  const double deltaLeft = leftPosition - lastLeftPosition;
  const double deltaRight = rightPosition - lastRightPosition;
  const double wheelDelta = ((deltaLeft + deltaRight) / 2);

  const double theta = (-gyroYaw - gyroOffset);
  const double deltaX = wheelDelta * std::cos(theta);
  const double deltaY = wheelDelta * std::sin(theta);

  w = WorldCoord{w.x + deltaX, w.y + deltaY, theta};
}
