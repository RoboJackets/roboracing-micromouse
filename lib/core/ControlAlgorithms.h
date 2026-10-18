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
  void reset() {
    accum = 0;
    lastError = 0;
    lastSetpoint = 0;
    initial = true;
  }
};

class TrapezoidalProfile {
public:
  struct State {
    double position = 0;
    double velocity = 0;
  };

  struct Constraints {
    double maxSpeed;
    double maxAccel;
  };

  Constraints cts;

  TrapezoidalProfile(Constraints constraints);

  State calculate(double dt, State setpoint, State goal) const;

  double totalTime(State setpoint, State goal) const;
};

struct MotorFeedForward {
  double ks;
  double kv;
  double ka;
  double lastVelocity = 0;

  MotorFeedForward(double ks, double kv, double ka);

  double calculate(double velocitySetpoint, double dt);
  void reset() { lastVelocity = 0; }
};
