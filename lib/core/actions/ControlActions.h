#pragma once
#include "Action.h"
#include "Commands.h"
#include "Constants.h"
#include "ControlAlgorithms.h"
#include <cmath>

struct DriveTimeAction : Action {
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  double totalTime = 0;
  double finalTime;
  double speed;

  DriveTimeAction(double time, double speed);

  void run(MazeMap &map, MouseIO &io) override;
  void end(MazeMap &map, MouseIO &io) override;
};

struct YawPIDAction : Action {
  bool canceled = false;
  double error = 0;
  int count = 0;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  PID p{rot90PIDConstants};
  double setpoint;

  YawPIDAction(double setpoint);

  void run(MazeMap &map, MouseIO &io) override;
  void end(MazeMap &map, MouseIO &io) override;
};

struct SysIDRampAction : Action {
  bool canceled = false;
  double totalTime = 0;
  double rampRate;
  double maxTime;

  SysIDRampAction(double rampRate = 0.03, double maxTime = 50.0);

  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  void run(MazeMap &map, MouseIO &io) override;
  void end(MazeMap &map, MouseIO &io) override;
};

struct RampVelocityAction : Action {
  bool canceled = false;
  double totalTime = 0;
  double rampRate;
  double maxTime;

  RampVelocityAction(double rampRate, double maxTime);

  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  void run(MazeMap &map, MouseIO &io) override;
  void end(MazeMap &map, MouseIO &io) override;
};

struct ProfiledDriveAction : Action {
  TrapezoidalProfile profile;
  double setpoint;
  double error;
  double angle;
  double measurement = 0;
  bool started = false;
  WorldCoord prevCoord;

  static constexpr double POS_TOL = 0.01; // 8 mm
  static constexpr double VEL_TOL = 0.06; // m/s
  double best = 0;
  ProfiledDriveAction(double setpoint, double angle, double finalVelocity,
                      double maxSpeed = 0.1);
  bool canceled = false;
  PID irPID = PID{IRadjust};
  PID gyroPID = PID{rot90PIDConstants};
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  void run(MazeMap &map, MouseIO &io) override;
  void end(MazeMap &map, MouseIO &io) override;
};

struct ProfiledRotationAction : Action {
  TrapezoidalProfile profile;
  bool canceled = false;
  bool started = false;
  double prevTheta = 0;
  double measurement = 0;
  double error;
  double setpoint;

  static constexpr double POS_TOL = 0.02; // rad
  static constexpr double VEL_TOL = 0.06; // m/s

  ProfiledRotationAction(double angle);

  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  void run(MazeMap &map, MouseIO &io) override;
  void end(MazeMap &map, MouseIO &io) override;
};

struct ProfiledCurveAction : Action {
  TrapezoidalProfile profile;
  bool canceled = false;
  PID irPID = PID{IRadjust};
  double measurement = 0;
  double irDelta = 0;
  double outerRatio;
  double radius;
  bool started = false;
  double prevTheta = 0;
  double setpoint;
  double error;

  static constexpr double POS_TOL = 0.008; // m (arc length)
  static constexpr double VEL_TOL = 0.06;  // m/s

  ProfiledCurveAction(double radius, double angle, double finalVelocity,
                      double maxSpeed = 0.0);

  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  void run(MazeMap &map, MouseIO &io) override;
  void end(MazeMap &map, MouseIO &io) override;
};
