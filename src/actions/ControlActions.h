#pragma once
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
  double error = 0;
  int count = 0;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  PID p{rot90PIDConstants};
  double setpoint;

  YawPIDAction(double setpoint) : setpoint(setpoint) {}

  void run(MouseState &s, MouseIO &io) override {
    double measure_r = io.getWorldCoord().theta;
    double error_raw = setpoint - measure_r;
    error = std::atan2(std::sin(error_raw), std::cos(error_raw));

    double avgSpeed = 0.5 * (std::abs(io.getDriveSpeedLeft()) +
                             std::abs(io.getDriveSpeedRight()));
    if (std::abs(error) < 4 * PI / 180 && avgSpeed < 0.06) {
      canceled = true;
      io.driveVoltage(0.0, 0.0);
      p.resetAccum();
      return;
    }

    double c = p.calculate(-error, 0, io.getDt());
    io.driveVelocity(c, -c);
  }

  void end(MouseState &s, MouseIO &io) override {
    Serial.print("ENDED!!!");
    io.driveVoltage(0.0, 0.0);
    io.resetPIDs();
    p.resetAccum();
  }
};
struct SysIDRampAction : Action {
  bool canceled = false;
  double totalTime = 0;
  double rampRate;
  double maxTime;

  SysIDRampAction(double rampRate = 0.03, double maxTime = 50.0)
      : rampRate(rampRate), maxTime(maxTime) {}

  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  void run(MouseState &s, MouseIO &io) override {
    totalTime += io.getDt();
    if (totalTime > maxTime) {
      canceled = true;
      return;
    }
    double voltage = totalTime * rampRate;
    io.driveVoltage(voltage, voltage);

    double speed = (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2.0;
    // Serial.print("VOLTS: ");
    // Serial.print(voltage, 4);
    // Serial.print(",SPEED: ");
    // Serial.println(speed, 4);
  }

  void end(MouseState &s, MouseIO &io) override { io.driveVoltage(0.0, 0.0); }
};

struct RampVelocityAction : Action {
  bool canceled = false;
  double totalTime = 0;
  double rampRate;
  double maxTime;

  RampVelocityAction(double rampRate, double maxTime)
      : rampRate(rampRate), maxTime(maxTime) {}

  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  void run(MouseState &s, MouseIO &io) override {
    totalTime += io.getDt();
    if (totalTime > maxTime) {
      canceled = true;
      return;
    }
    double velocity = totalTime * rampRate;
    // Serial.print(velocity);
    io.driveVelocity(velocity, velocity);
  }

  void end(MouseState &s, MouseIO &io) override { io.driveVoltage(0.0, 0.0); }
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
  static constexpr double VEL_TOL = 0.06;  // m/s
  double best = 0;
  ProfiledDriveAction(double setpoint, double angle, double finalVelocity)
      : profile({0.1, MAX_ACCEL_M_S2 / 2, 0, finalVelocity,
                 profilePIDConstants, setpoint}),
        setpoint(setpoint), error(setpoint), angle(angle) {}
  bool canceled = false;
  PID irPID = PID{IRadjust};
  PID gyroPID = PID{rot90PIDConstants};
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  double irDelta = 0;
  void run(MouseState &s, MouseIO &io) override {
    // if (io.getAverageSensorState()[0].hypot() < 0.1 ||
    //     io.getAverageSensorState()[1].hypot() < 0.1) {
    //   canceled = true;
    // }
    // Serial.printf("ERROR: %0.2f\n", error);
    double avgSpeed = 0.5 * (std::abs(io.getDriveSpeedLeft()) +
                             std::abs(io.getDriveSpeedRight()));
    bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
    if (std::abs(error) < POS_TOL && velOk) {
      canceled = true;
      return;
    }
    if (canceled) {
      return;
    }
    WorldCoord w = io.getWorldCoord();
    if (!started) {
      prevCoord = w;
      double vForward = (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2.0;
      profile.initalVelocity = vForward * std::cos(w.theta - angle);
      started = true;
    }
    irDelta =
        io.getSensorState().at(2).hypot() - io.getSensorState().at(3).hypot();
    double dx = w.x - prevCoord.x;
    double dy = w.y - prevCoord.y;
    measurement += dx * std::cos(angle) + dy * std::sin(angle);
    prevCoord = w;
    error = setpoint - measurement;
    if ((io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2 > best) {
      best = (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2;
      // Serial.println(best);
    }
    double v = profile.calculate(io.getDt(), measurement);
    // bool hasBothIR = io.getSensorState().at(2).hypot() < 0.18 &&
    //                  io.getSensorState().at(3).hypot() < 0.18;
    bool hasBothIR = false;
    double c;
    double gyroError = angle - w.theta;
    gyroError = std::atan2(std::sin(gyroError), std::cos(gyroError));
    c = gyroPID.calculate(-gyroError, 0, io.getDt());
    if (hasBothIR) {
      c += irPID.calculate(irDelta, 0, io.getDt());
    }
    // Serial.println(v);
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

  ProfiledRotationAction(double angle)
      : profile({MAX_ROT_SPEED_RAD_S, MAX_ROT_SPEED_RAD_S2, 0, 0,
                 profilePIDConstants, angle}),
        setpoint(angle), error(angle) {}

  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  void run(MouseState &s, MouseIO &io) override {
    double avgSpeed = 0.5 * (std::abs(io.getDriveSpeedLeft()) +
                             std::abs(io.getDriveSpeedRight()));
    bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
    if (std::abs(error) < POS_TOL && velOk) {
      canceled = true;
      return;
    }
    if (canceled) {
      return;
    }
    WorldCoord w = io.getWorldCoord();
    if (!started) {
      prevTheta = w.theta;
      started = true;
    }
    double dTheta = w.theta - prevTheta;
    dTheta = std::atan2(std::sin(dTheta), std::cos(dTheta));
    measurement += dTheta;
    prevTheta = w.theta;
    error = setpoint - measurement;
    double omega = profile.calculate(io.getDt(), measurement);
    double wheelSpeed = omega * WHEEL_SEPERATION_M / 2.0;
    io.driveVelocity(wheelSpeed, -wheelSpeed);
  }

  void end(MouseState &s, MouseIO &io) override { io.driveVoltage(0, 0); }
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

  ProfiledCurveAction(double radius, double angle, double finalVelocity)
      : profile({0.2, MAX_ACCEL_M_S2, 0, finalVelocity,
                 profilePIDConstants, radius * std::abs(angle)}),
        outerRatio((radius + WHEEL_SEPERATION_M / 2.0) / radius),
        radius(radius), setpoint(radius * angle), error(setpoint) {}

  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  void run(MouseState &s, MouseIO &io) override {
    // if (io.getAverageSensorState()[0].hypot() < 0.1 ||
    //     io.getAverageSensorState()[1].hypot() < 0.1) {
    //   canceled = true;
    // }
    double avgSpeed = 0.5 * (std::abs(io.getDriveSpeedLeft()) +
                             std::abs(io.getDriveSpeedRight()));
    bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
    if (std::abs(error) < POS_TOL && velOk) {
      canceled = true;
      return;
    }
    if (canceled) {
      return;
    }
    WorldCoord w = io.getWorldCoord();
    if (!started) {
      prevTheta = w.theta;
      double vForward = (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2.0;
      profile.initalVelocity = vForward;
      started = true;
    }
    double dTheta = w.theta - prevTheta;
    dTheta = std::atan2(std::sin(dTheta), std::cos(dTheta));
    measurement += dTheta * radius;
    prevTheta = w.theta;
    error = setpoint - measurement;
    double v = profile.calculate(io.getDt(), std::abs(measurement));
    // double c =
    //     std::isinf(irDelta) ? 0.0 : irPID.calculate(irDelta, 0, io.getDt());
    double c = 0;
    double vOuter = v * outerRatio + c;
    double vInner = v / outerRatio - c;
    (setpoint < 0) ? io.driveVelocity(vInner, vOuter)
                   : io.driveVelocity(vOuter, vInner);
  }

  void end(MouseState &s, MouseIO &io) override {
    if (profile.finalVelocity == 0) {
      io.driveVoltage(0, 0);
    } else {
      io.driveVelocity(profile.finalVelocity, profile.finalVelocity);
    }
    irPID.resetAccum();
  }
};