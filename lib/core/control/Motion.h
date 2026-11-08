#pragma once

#include "Types.h"
#include "ControlAlgorithms.h"

class MotionProfile {
  TrapezoidalProfile profile;
  TrapezoidalProfile::State target;
  TrapezoidalProfile::State ref;
  static constexpr double POS_EPS = 1e-9;
  bool done = true;
  double dir = 1.0;

public:
  MotionProfile();

  void startMove(double position, double finalVelocity, TrapezoidalProfile::Constraints constraints);
  void hold();
  void update(double dt);
  void reset(double measured);

  double position() { return ref.position; }
  double velocity() { return ref.velocity; }
  bool finished() const { return done; }
};

struct MotionController {
  MotionProfile forward;
  PID forwardPID;

  MotionProfile rotation;
  PID rotationPID;

  double forwardError = 0;
  double rotationError = 0;

  const double driveTol;
  const double rotTol;
  const double speedTol;
  const double wheelSeparation;

  bool enabled = false;

  MotionController(PIDConstants forward, PIDConstants rotation, double driveTol, double rotTol, double speedTol, double wheelSeparation);

  WheelSpeeds update(double dt, double distance, double angle);
  void reset(double distance, double angle);
  bool settled(double measuredSpeed) const;
};
