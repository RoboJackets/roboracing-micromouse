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
    Serial.println("run");
    io.drive(0.5, 0.5);
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
    double error = std::fmod((setpoint - io.getGyroYaw()), 180) - 180;
    double c = p.calculate(error, 0, io.getDt());
    io.drive(c, -c);
  }

  void end(MouseState &s, MouseIO &io) override {
    io.drive(0.0, 0.0);
    p.resetAccum();
  }
};