#include "Robot.h"

#include <algorithm>

void Robot::init() {
  io.init();
  lastNow = io.now();
}

void Robot::update() {
  const double n = io.now();
  cachedDt = std::max(n - lastNow, 1e-6);
  lastNow = n;

  sensors.update(io.irMeters());
  const double yaw = io.gyroYaw();
  const double left = io.leftMeters();
  const double right = io.rightMeters();
  odom.update(left, right, yaw, cachedDt);
}
