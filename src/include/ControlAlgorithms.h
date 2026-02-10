#pragma once
#include <cmath>
struct TrapezoidalProfile {
  double maxSpeed;
  double maxAccel;
  TrapezoidalProfile(double maxSpeed, double maxAccel)
      : maxSpeed(maxSpeed), maxAccel(maxAccel) {}

  double distance = 0;
  void setDistance(double dist) { distance = dist; }

  
};