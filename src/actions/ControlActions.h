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
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  PID p{rot90PIDConstants};
  double setpoint;

  YawPIDAction(double setpoint) : setpoint(setpoint) {}

  void run(MouseState &s, MouseIO &io) override {
    double setpoint_r = setpoint;
    double measure_r = io.getWorldCoord().theta;
    double error_raw = setpoint_r - measure_r;
    error = std::atan2(std::sin(error_raw), std::cos(error_raw));
    double c = p.calculate(-error, 0, io.getDt());
    io.driveVelocity(c, -c);
    // Serial.print("VOLTS: ");
    // Serial.print(c);
    // Serial.print("    ");
    // Serial.print("GYRO: ");
    // Serial.println(measure_r);
  }

  void end(MouseState &s, MouseIO &io) override {
    Serial.print("ENDED!!!");
    io.driveVoltage(0.0, 0.0);
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
    Serial.print("VOLTS: ");
    Serial.print(voltage, 4);
    Serial.print(",SPEED: ");
    Serial.println(speed, 4);
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
    Serial.print(velocity);
    io.driveVelocity(velocity, velocity);
  }

  void end(MouseState &s, MouseIO &io) override { io.driveVoltage(0.0, 0.0); }
};

struct ProfiledDriveAction : Action {
  TrapezoidalProfile profile;
  double error;
  double setpoint;
  double angle;
  double measurement = 0;
  bool started = false;
  WorldCoord prevCoord;
  double best = 0;
  ProfiledDriveAction(double setpoint, double angle, double initalVelocity,
                      double finalVelocity)
      : profile({1, MAX_ACCEL_M_S2, initalVelocity, finalVelocity,
                 profilePIDConstants, setpoint}),
        setpoint(setpoint), angle(angle), error(setpoint) {}
  bool canceled = false;
  PID irPID = PID{IRadjust};
  PID gyroPID = PID{rot90PIDConstants};
  void cancel() override { canceled = true; }
  bool completed() const override { return std::abs(error) < 0.03 || canceled; }
  double irDelta = 0;
  void run(MouseState &s, MouseIO &io) override {
    // if (io.getAverageSensorState()[0].hypot() < 0.1 ||
    //     io.getAverageSensorState()[1].hypot() < 0.1) {
    //   canceled = true;
    // }
    // Serial.printf("ERROR: %0.2f\n", error);
    if (completed()) {
      if (!canceled) {
        Serial.println("YES YES");
      }
      canceled = true;
    }
    if (canceled) {
      io.driveVoltage(0, 0);
      return;
    }
    WorldCoord w = io.getWorldCoord();
    if (!started) {
      prevCoord = w;
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
      Serial.println(best);
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
  double error;
  double setpoint;

  ProfiledCurveAction(double radius, double angle, double initialVelocity,
                      double finalVelocity)
      : profile({CURVE_VELOCITY, MAX_ACCEL_M_S2, initialVelocity, finalVelocity,
                 profilePIDConstants, radius * std::abs(angle)}),
        outerRatio((radius + WHEEL_SEPERATION_M / 2.0) / radius),
        radius(radius), setpoint(radius * angle), error(setpoint) {}

  void cancel() override { canceled = true; }
  bool completed() const override { return std::abs(error) < 0.03 || canceled; }

  void run(MouseState &s, MouseIO &io) override {
    if (io.getAverageSensorState()[0].hypot() < 0.1 ||
        io.getAverageSensorState()[1].hypot() < 0.1) {
      canceled = true;
    }
    WorldCoord w = io.getWorldCoord();
    if (!started) {
      prevTheta = w.theta;
      started = true;
    }
    double dTheta = w.theta - prevTheta;
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
    profile.finalVelocity == 0
        ? io.driveVoltage(0, 0)
        : io.driveVelocity(profile.finalVelocity, profile.finalVelocity);
    irPID.resetAccum();
  }
};