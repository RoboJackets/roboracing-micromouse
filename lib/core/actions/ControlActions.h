#pragma once
#include "Action.h"
#include "Constants.h"
#include "ControlAlgorithms.h"
#include "Tuning.h"
#include "Types.h"
#include <cmath>

struct StopAction : Action {
  void run(Robot &r) override;
};

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
  const Translation2d start;
  const Translation2d target;
  const Translation2d dir;
  const double lineAngle;

  const double v0; // initial speed
  const double vf; // final speed
  
  double time = 0;
  TrapezoidalProfile profile;
  TrapezoidalProfile::Constraints constraints;

  PID distPID = PID{profilePIDConstants};
  PID irPID = PID{IRadjust};
  PID gyroPID = PID{rot90PIDConstants};
    
  ProfiledDriveAction(const Translation2d start, const Translation2d target, const TrapezoidalProfile::Constraints);
  ProfiledDriveAction(const Translation2d start, const Translation2d target, const TrapezoidalProfile::Constraints, const double v0, const double vf);

  void run(Robot &r) override;
  void begin(Robot &r) override;
  void end(Robot &r) override;
};

// Pivot in place to an absolute heading using the robot's motion controller.
struct ProfiledRotationAction : Action {
  const double target; // absolute heading, rad
  const TrapezoidalProfile::Constraints constraints; // rad/s, rad/s^2

  ProfiledRotationAction(double target);

  void begin(Robot &r) override;
  void run(Robot &r) override;
  void end(Robot &r) override;
};

struct ProfiledCurveAction : Action {
  TrapezoidalProfile profile;
  TrapezoidalProfile::State profileSetpoint{};
  PID errorPid = PID{profilePIDConstants};
  PID irPID = PID{IRadjust};
  double finalVelocity;
  double measurement = 0;
  double radius;
  bool started = false;
  double prevTheta = 0;
  double setpoint;
  double error;

  ProfiledCurveAction(double radius, double angle, double finalVelocity,
                      double maxSpeed = 0.0);

  void run(Robot &r) override;
  void end(Robot &r) override;
};
