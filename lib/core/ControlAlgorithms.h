#pragma once
#include <algorithm>
#include <cmath>

struct PIDConstants {
  double p = 0;
  double i = 0;
  double d = 0;
  double maxAccum = 1.0;
};

inline PIDConstants rot90PIDConstants{0.5, 0, 0};
inline PIDConstants velocityPIDConstants{1.1, 0.3, 0.0, 0.3};
inline PIDConstants profilePIDConstants{4, 0, 0};
inline PIDConstants IRadjust{0.3, 0, 0};

class PID {
  double p = 0;
  double i = 0;
  double d = 0;
  double maxAccum = 1.0;
  double lastError = 0;
  bool initial = true;
  double lastSetpoint = 0;
  double accum = 0;

public:
  PID(PIDConstants constants);
  double calculate(double measurement, double setpoint, double dt);
  void resetAccum() { accum = 0; }
};

struct TrapezoidalProfile {
  double maxSpeed, maxAccel, initialVelocity, finalVelocity;
  PIDConstants pidConstants;
  double setpoint;
  double time = 0;
  double startMeasurement = 0;
  bool started = false;
  PID errorPid = PID{pidConstants};

  TrapezoidalProfile(double maxSpeed, double maxAccel, double initialVelocity,
                     double finalVelocity, PIDConstants pidConstants,
                     double setpoint);

  double calculate(double dt, double measurement);
  void reset();

  static double totalTime(double maxAccel, double maxSpeed, double distance);
};

struct MotorFeedForward {
  double ks;
  double kv;
  double ka;
  double lastVelocity = 0;

  MotorFeedForward(double ks, double kv, double ka);

  double calculate(double velocitySetpoint, double dt);
};
