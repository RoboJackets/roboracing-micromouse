#include "Action.h"
#include "CommandGenerator.h"
#include "Commands.h"
#include "pid.hpp"
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
    io.drive(1, 1);
  }
  void end(MouseState &s, MouseIO &io) override { io.drive(0.0, 0.0); }
};

struct YawPIDAction : Action {
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  PID p{rot90PID};
  double setpoint;

  YawPIDAction(double setpoint) : setpoint(setpoint) {}

  void run(MouseState &s, MouseIO &io) override {
    double setpoint_r = setpoint * (PI / 180);
    double measure_r = io.getGyroYaw() * (PI / 180);
    double error_raw = setpoint_r - measure_r;
    double error =
        std::atan2(std::sin(error_raw), std::cos(error_raw)) * (180 / PI);
    Serial.print(error);
    Serial.print("     ");
    Serial.print(io.getGyroYaw());
    Serial.print("     ");
    double c = p.calculate(error, 0, io.getDt());
    Serial.println(c);
    io.drive(c, -c);
  }

  void end(MouseState &s, MouseIO &io) override {
    io.drive(0.0, 0.0);
    p.resetAccum();
  }
};