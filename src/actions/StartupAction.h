#pragma once
#include "Action.h"
#include "ControlAlgorithms.h"
#include <Constants.h>
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
    io.setGyroOffset(io.getGyroYaw() - 90);
    io.setWorldCoord(WorldCoord{left.x, ROBOT_LENGTH / 2});
    canceled = true;
  }
  void end(MouseState &s, MouseIO &io) override {}
};