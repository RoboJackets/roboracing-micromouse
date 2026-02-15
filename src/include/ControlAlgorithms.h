#pragma once
#include "pid.hpp"
#include <Action.h>
#include <cmath>
#include <tuple>
#include <vector>

struct TrapezoidalProfile {
  double maxSpeed;
  double maxAccel;
  PIDConstants pidConstants;
  double setpoint;
  double time = 0;
  PID errorPid = PID{pidConstants};
  TrapezoidalProfile(double maxSpeed, double maxAccel,
                     PIDConstants pidConstants, double setpoint)
      : maxSpeed(maxSpeed), maxAccel(maxAccel), pidConstants(pidConstants),
        setpoint(setpoint) {}

  double calculate(double dt, double measurement) {
    time += dt;

    double distance = std::abs(setpoint - measurement);
    double direction = setpoint - measurement >= 0.0 ? 1.0 : -1.0;

    double accelTime = maxSpeed / maxAccel;
    double accelDistance = 0.5 * maxAccel * accelTime * accelTime;

    double velocity = 0.0;
    double position = 0.0;

    if (distance >= 2.0 * accelDistance) {
      double cruiseTime = (distance - 2.0 * accelDistance) / maxSpeed;
      double totalTime = 2.0 * accelTime + cruiseTime;

      if (time < accelTime) {
        velocity = maxAccel * time;
        position = 0.5 * maxAccel * time * time;
      } else if (time < accelTime + cruiseTime) {
        double t = time - accelTime;
        velocity = maxSpeed;
        position = accelDistance + maxSpeed * t;
      } else if (time < totalTime) {
        double t = time - accelTime - cruiseTime;
        velocity = maxSpeed - maxAccel * t;
        position = accelDistance + maxSpeed * cruiseTime + maxSpeed * t -
                   0.5 * maxAccel * t * t;
      } else {
        velocity = 0.0;
        position = distance;
      }
    } else {
      double peakVelocity = std::sqrt(distance * maxAccel);
      double peakTime = peakVelocity / maxAccel;
      double totalTime = 2.0 * peakTime;

      if (time < peakTime) {
        velocity = maxAccel * time;
        position = 0.5 * maxAccel * time * time;
      } else if (time < totalTime) {
        double t = time - peakTime;
        velocity = peakVelocity - maxAccel * t;
        position = 0.5 * maxAccel * peakTime * peakTime + peakVelocity * t -
                   0.5 * maxAccel * t * t;
      } else {
        velocity = 0.0;
        position = distance;
      }
    }

    velocity *= direction;
    position *= direction;

    return velocity + errorPid.calculate(measurement, position, dt);
  }

  void reset() { time = 0; }

  static double totalTime(double maxAccel, double maxSpeed, double distance) {
    double D = std::abs(distance);

    double accelTime = maxSpeed / maxAccel;
    double accelDistance = 0.5 * maxAccel * accelTime * accelTime;

    if (D >= 2.0 * accelDistance) {
      double cruiseTime = (D - 2.0 * accelDistance) / maxSpeed;
      return 2.0 * accelTime + cruiseTime;
    } else {
      double peakVelocity = std::sqrt(D * maxAccel);
      double peakTime = peakVelocity / maxAccel;
      return 2.0 * peakTime;
    }
  }
};

struct MotorFeedForward {
  double ks;
  double kv;
  double ka;
  double lastVelocity = 0;

  MotorFeedForward(double ks, double kv, double ka) : ks(ks), kv(kv), ka(ka) {}

  double calculate(double velocitySetpoint, double velocityMeasurement,
                   double dt) {
    double sign = (0 < velocityMeasurement) - (velocityMeasurement < 0);
    double voltage = ks * sign + kv * velocityMeasurement * ka *
                                     (velocityMeasurement - lastVelocity) / dt;
    lastVelocity = velocityMeasurement;
    return voltage;
  }
};

struct SysIDAction : Action {
  bool canceled = false;
  double quasistaticTime, dynamicTime;
  double voltAccel, voltDynamic;

  std::vector<std::tuple<double, double>> quasiForward = {};
  std::vector<std::tuple<double, double>> quasiBackward = {};
  std::vector<std::tuple<double, double>> dynamicForward = {};
  std::vector<std::tuple<double, double>> dynamicBackward = {};

  double totalTime = 0;

  SysIDAction(double quasistaticTime, double dynamicTime)
      : quasistaticTime(quasistaticTime), dynamicTime(dynamicTime) {}
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  void run(MouseState &s, MouseIO &io) override {
    double t0 = totalTime + quasistaticTime;
    double t1 = t0 + 3;
    double t2 = t1 + quasistaticTime;
    double t3 = t2 + 3;
    double t4 = t3 + dynamicTime;
    double t5 = t4 + 3;
    double t6 = t5 + dynamicTime;
    if (totalTime <= t0) {
      quasiForward.push_back(
          {totalTime, (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2});
      io.driveVoltage(totalTime * voltAccel, totalTime * voltAccel);
    } else if (totalTime >= t1 && totalTime <= t2) {
      quasiBackward.push_back(
          {totalTime, (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2});
      io.driveVoltage(-(totalTime - t1) * voltAccel,
                      -(totalTime - t1) * voltAccel);
    } else if (totalTime >= t3 && totalTime <= t4) {
      dynamicForward.push_back(
          {totalTime, (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2});
      io.driveVoltage(voltDynamic, voltDynamic);
    } else if (totalTime >= t5 && totalTime <= t6) {
      dynamicBackward.push_back(
          {totalTime, (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2});
      io.driveVoltage(-voltDynamic, -voltDynamic);
    } else if (totalTime > t6 + 1) {
      cancel();
    } else {
      io.driveVoltage(0, 0);
    }
    totalTime += io.getDt();
  }
  void end(MouseState &s, MouseIO &io) override {
    io.driveVoltage(0.0, 0.0); 
  }
};