#include "Action.h"
#include "CommandGenerator.h"
#include "Commands.h"
#include <Arduino.h>
#include <cmath>

struct DriveTimeAction : Action {
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  double totalTime = 0;
  double finalTime;
  double speed;

  DriveTimeAction(double time, double speed) : finalTime(time), speed(speed) {}

  void run(MouseState &s, MouseIO &io) override {
    totalTime += io.getDt();
    if (totalTime > finalTime) {
      canceled = true;
      return;
    }
    io.driveVoltage(1, 1);
  }
  void end(MouseState &s, MouseIO &io) override { io.driveVoltage(0.0, 0.0); }
};

struct YawPIDAction : Action {
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  PID p{rot90PIDConstants};
  double setpoint;

  YawPIDAction(double setpoint) : setpoint(setpoint) {}

  void run(MouseState &s, MouseIO &io) override {
    double setpoint_r = setpoint * (PI / 180);
    double measure_r = io.getGyroYaw();
    double error_raw = setpoint_r - measure_r;
    double error =
        std::atan2(std::sin(error_raw), std::cos(error_raw));
    Serial.print(error);
    Serial.print("     ");
    Serial.print(io.getGyroYaw());
    Serial.print("     ");
    double c = p.calculate(error, 0, io.getDt());
    Serial.println(c);
    io.driveVoltage(c, -c);
  }

  void end(MouseState &s, MouseIO &io) override {
    io.driveVoltage(0.0, 0.0);
    p.resetAccum();
  }
};
struct ProfiledDriveAction : Action {
  TrapezoidalProfile profile;
  ProfiledDriveAction(double setpoint, double initalVelocity,
                      double finalVelocity)
      : profile({MAX_SPEED_M_S, MAX_ACCEL_M_S2, initalVelocity, finalVelocity,
                 profilePIDConstants, setpoint}) {}
  bool canceled = false;
  PID irPID = PID{IRadjust};
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  double measurement = 0;
  double irDelta = 0;
  void setMeasurement(double m) { measurement = m; }
  void setIRDelta(double i) { irDelta = i; }
  void run(MouseState &s, MouseIO &io) override {
    double v = profile.calculate(io.getDt(), measurement);
    double c = irPID.calculate(irDelta, 0, io.getDt());
    io.driveVelocity(v + c, v - c);
  }
  void end(MouseState &s, MouseIO &io) override {
    if (profile.finalVelocity == 0) {
      io.driveVoltage(0, 0);
    } else {
      io.driveVelocity(profile.finalVelocity, profile.finalVelocity);
    }
  }
};
struct ProfiledCurveAction : Action {
  TrapezoidalProfile profile;
  bool canceled = false;
  PID irPID = PID{IRadjust};
  double measurement = 0;
  double irDelta = 0;
  bool turnLeft;
  double outerRatio;

  ProfiledCurveAction(double radius, double angle, bool turnLeft,
                      double initialVelocity, double finalVelocity)
      : profile({CURVE_VELOCITY, MAX_ACCEL_M_S2, initialVelocity, finalVelocity,
                 profilePIDConstants, radius * angle}),
        turnLeft(turnLeft),
        outerRatio((radius + WHEEL_SEPERATION_M / 2.0) / radius) {}

  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  void setMeasurement(double m) { measurement = m; }
  void setIRDelta(double i) { irDelta = i; }

  void run(MouseState &s, MouseIO &io) override {
    double v = profile.calculate(io.getDt(), measurement);
    double c =
        std::isinf(irDelta) ? 0.0 : irPID.calculate(irDelta, 0, io.getDt());
    double vOuter = v * outerRatio + c;
    double vInner = v / outerRatio - c;
    turnLeft ? io.driveVelocity(vInner, vOuter)
             : io.driveVelocity(vOuter, vInner);
  }

  void end(MouseState &s, MouseIO &io) override {
    profile.finalVelocity == 0
        ? io.driveVoltage(0, 0)
        : io.driveVelocity(profile.finalVelocity, profile.finalVelocity);
    irPID.resetAccum();
  }
};