#pragma once
#include "pid.hpp"
#include <cmath>

struct TrapezoidalProfile {
  double maxSpeed;
  double maxAccel;
  PIDConstants pidConstants;
  double setpoint;
  double time = 0;
  TrapezoidalProfile(double maxSpeed, double maxAccel,
                     PIDConstants pidConstants, double setpoint)
      : maxSpeed(maxSpeed), maxAccel(maxAccel), pidConstants(pidConstants),
        setpoint(setpoint) {}

  double calculate(double dt) {
    time += dt;
    double accelTime = maxSpeed / maxAccel;
    double v = (time <= accelTime) ? maxAccel * time : maxSpeed;
    double x = (time <= accelTime) ? 0.5 * maxAccel * time * time
                                   : maxAccel * accelTime * time -
                                         0.5 * maxAccel * accelTime * accelTime;
    return x;
  }
  void reset() { time = 0; }
};