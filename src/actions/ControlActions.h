#pragma once
#include "Action.h"
#include "CommandGenerator.h"
#include "Commands.h"
#include <Arduino.h>
#include <cmath>
#include <numeric>



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
  bool completed() const override { return std::abs(error) < 0.01 || canceled; }

  PID p{rot90PIDConstants};
  double setpoint;

  YawPIDAction(double setpoint) : setpoint(setpoint) {}

  void run(MouseState &s, MouseIO &io) override {
    double setpoint_r = setpoint;
    double measure_r = io.getWorldCoord().theta;
    double error_raw = setpoint_r - measure_r;
    error = std::atan2(std::sin(error_raw), std::cos(error_raw));
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
  double error = 0;
  double setpoint;
  double angle;
  double measurement = 0;
  bool started = false;
  std::vector<double> distancesL{};
  std::vector<double> distancesR{};
  WorldCoord prevCoord;
  ProfiledDriveAction(double setpoint, double angle, double initalVelocity,
                      double finalVelocity)
      : profile({MAX_SPEED_M_S, MAX_ACCEL_M_S2, initalVelocity, finalVelocity,
                 profilePIDConstants, setpoint}),
        setpoint(setpoint), angle(angle) {}
  bool canceled = false;
  PID irPID = PID{IRadjust};
  void cancel() override { canceled = true; }
  bool completed() const override {
    return std::abs(error) < 0.005 || canceled;
  }
  double irDelta = 0;
  void run(MouseState &s, MouseIO &io) override {
    if (cancelAvoidWall(s, io)) {
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
  bool cancelAvoidWall(MouseState &state, MouseIO &io) {
    std::vector<WorldCoord> readings = io.getSensorState();
    double wallThresh = .2;
    double distFrontFromWallL = sqrt(pow(readings[0].x - state.x, 2) + pow(readings[0].y - state.y, 2));
    double distFrontFromWallR = sqrt(pow(readings[1].x - state.x, 2) + pow(readings[1].y - state.y, 2));
    if (distancesL.size() == 5) {
      distancesL.pop_back();
    }
    distancesL.insert(distancesL.begin(), distFrontFromWallL);
    if (distancesR.size() == 5) {
      distancesR.pop_back();
    }
    distancesR.insert(distancesR.begin(), distFrontFromWallR);
    bool canceled = std::accumulate(distancesR.begin(), distancesR.end(), 0.0)/distancesR.size() < wallThresh ||
                    std::accumulate(distancesL.begin(), distancesL.end(), 0.0)/distancesL.size() < wallThresh;
    return canceled;
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
  double error = 0;
  double setpoint;
  std::vector<double> distancesL{};
  std::vector<double> distancesR{};

  ProfiledCurveAction(double radius, double angle, double initialVelocity,
                      double finalVelocity)
      : profile({CURVE_VELOCITY, MAX_ACCEL_M_S2, initialVelocity, finalVelocity,
                 profilePIDConstants, radius * std::abs(angle)}),
        outerRatio((radius + WHEEL_SEPERATION_M / 2.0) / radius),
        radius(radius), setpoint(radius * angle) {}

  void cancel() override { canceled = true; }
  bool completed() const override {
    return std::abs(error) < 0.005 || canceled;
  }

  void run(MouseState &s, MouseIO &io) override {
    if (cancelAvoidWall(s, io)) {
      return;
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

  bool cancelAvoidWall(MouseState &state, MouseIO &io) {
    std::vector<WorldCoord> readings = io.getSensorState();
    double wallThresh = .2;
    double distFrontFromWallL = sqrt(pow(readings[0].x - state.x, 2) + pow(readings[0].y - state.y, 2));
    double distFrontFromWallR = sqrt(pow(readings[1].x - state.x, 2) + pow(readings[1].y - state.y, 2));
    if (distancesL.size() == 5) {
      distancesL.pop_back();
    }
    distancesL.insert(distancesL.begin(), distFrontFromWallL);
    if (distancesR.size() == 5) {
      distancesR.pop_back();
    }
    distancesR.insert(distancesR.begin(), distFrontFromWallR);
    bool canceled = std::accumulate(distancesR.begin(), distancesR.end(), 0.0)/distancesR.size() < wallThresh ||
                    std::accumulate(distancesL.begin(), distancesL.end(), 0.0)/distancesL.size() < wallThresh;
    return canceled;
  }
};