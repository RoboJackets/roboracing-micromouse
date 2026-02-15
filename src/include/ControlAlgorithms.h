#pragma once
#include "pid.hpp"
#include <cmath>

struct TrapezoidalProfile {
  double maxSpeed;
  double maxAccel;
  PIDConstants pidConstants;
  double setpoint;
  double time = 0;
  PID errorPid = PID{pidConstants};
  TrapezoidalProfile(double maxSpeed, double maxAccel,
                     PIDConstants pidConstants, double setpoint)
      : maxSpeed(maxSpeed), maxAccel(maxAccel), pidConstants(pidConstants),
        setpoint(setpoint) {}

  double calculate(double dt, double measurement) {
    time += dt;

    double distance = std::abs(setpoint - measurement);
    double direction = setpoint - measurement >= 0.0 ? 1.0 : -1.0;

    double accelTime = maxSpeed / maxAccel;
    double accelDistance = 0.5 * maxAccel * accelTime * accelTime;

    double velocity = 0.0;
    double position = 0.0;

    if (distance >= 2.0 * accelDistance) {
      double cruiseTime = (distance - 2.0 * accelDistance) / maxSpeed;
      double totalTime = 2.0 * accelTime + cruiseTime;

      if (time < accelTime) {
        velocity = maxAccel * time;
        position = 0.5 * maxAccel * time * time;
      } else if (time < accelTime + cruiseTime) {
        double t = time - accelTime;
        velocity = maxSpeed;
        position = accelDistance + maxSpeed * t;
      } else if (time < totalTime) {
        double t = time - accelTime - cruiseTime;
        velocity = maxSpeed - maxAccel * t;
        position = accelDistance + maxSpeed * cruiseTime + maxSpeed * t -
                   0.5 * maxAccel * t * t;
      } else {
        velocity = 0.0;
        position = distance;
      }
    } else {
      double peakVelocity = std::sqrt(distance * maxAccel);
      double peakTime = peakVelocity / maxAccel;
      double totalTime = 2.0 * peakTime;

      if (time < peakTime) {
        velocity = maxAccel * time;
        position = 0.5 * maxAccel * time * time;
      } else if (time < totalTime) {
        double t = time - peakTime;
        velocity = peakVelocity - maxAccel * t;
        position = 0.5 * maxAccel * peakTime * peakTime + peakVelocity * t -
                   0.5 * maxAccel * t * t;
      } else {
        velocity = 0.0;
        position = distance;
      }
    }

    velocity *= direction;
    position *= direction;

    return velocity + errorPid.calculate(measurement, position, dt);
  }

  void reset() { time = 0; }
};