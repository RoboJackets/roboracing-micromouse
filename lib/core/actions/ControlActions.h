#pragma once
#include "Action.h"
#include "Commands.h"
#include "Constants.h"
#include "ControlAlgorithms.h"
#include "Types.h"
#include <cmath>

struct DriveTimeAction : Action {
  double totalTime = 0;
  double finalTime;
  double speed;

  DriveTimeAction(double time, double speed);

  void run(Robot &r) override;
  void end(Robot &r) override;
};

struct YawPIDAction : Action {
  double error = 0;
  int count = 0;

  PID p{rot90PIDConstants};
  double setpoint;

  YawPIDAction(double setpoint);

  void run(Robot &r) override;
  void end(Robot &r) override;
};

struct ProfiledDriveAction : Action {
  TrapezoidalProfile profile;
  double setpoint;
  double error;
  double angle;
  double measurement = 0;
  bool started = false;
  WorldCoord prevCoord;

  static constexpr double POS_TOL = 0.01; // 10 mm
  static constexpr double VEL_TOL = 0.06; // m/s
  ProfiledDriveAction(double setpoint, double angle, double finalVelocity,
                      double maxSpeed = 0.1);
  PID irPID = PID{IRadjust};
  PID gyroPID = PID{rot90PIDConstants};

  void run(Robot &r) override;
  void end(Robot &r) override;
};

struct ProfiledRotationAction : Action {
  TrapezoidalProfile profile;
  bool started = false;
  double prevTheta = 0;
  double measurement = 0;
  double error;
  double setpoint;

  static constexpr double POS_TOL = 0.02; // rad
  static constexpr double VEL_TOL = 0.06; // m/s

  ProfiledRotationAction(double angle);

  void run(Robot &r) override;
  void end(Robot &r) override;
};

struct ProfiledCurveAction : Action {
  TrapezoidalProfile profile;
  PID irPID = PID{IRadjust};
  double measurement = 0;
  double radius;
  bool started = false;
  double prevTheta = 0;
  double setpoint;
  double error;

  static constexpr double POS_TOL = 0.008; // m (arc length)
  static constexpr double VEL_TOL = 0.06;  // m/s

  ProfiledCurveAction(double radius, double angle, double finalVelocity,
                      double maxSpeed = 0.0);

  void run(Robot &r) override;
  void end(Robot &r) override;
};
