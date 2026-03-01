#pragma once
#include "Action.h"
#include "ControlAlgorithms.h"
#include <IRSensor.h>

struct StartupAction : Action {
  IRSensor left;
  IRSensor right;
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  PID p = PID{IRadjust};
  void run(MouseState &s, MouseIO &io) override {
    WorldCoord left = io.getSensorState().at(2);
    WorldCoord right = io.getSensorState().at(3);
    double e = left.x - right.x;
    double c = -p.calculate(e, 0, io.getDt());
    io.driveVoltage(c, -c);
  }
  void end(MouseState &s, MouseIO &io) override {
    io.driveVoltage(0.0, 0.0);
    io.setGyroOffset(io.getGyroYaw());
  }
};