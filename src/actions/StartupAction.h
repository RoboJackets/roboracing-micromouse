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
    io.setGyroOffset(io.getGyroYaw() - M_PI / 2.0);
    io.setWorldCoord(WorldCoord{-left.x, ROBOT_LENGTH / 2});
    canceled = true;
    Serial.print("a");
    io.allowUpdates(true);
    Serial.print("b");
  }
  void end(MouseState &s, MouseIO &io) override {}
};
struct DelayAction : Action {
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  double time = 0;
  double runTime;
  DelayAction(double runTime) : runTime(runTime) {}
  void run(MouseState &s, MouseIO &io) override {
    time += io.getDt();
    // Serial.println(time);
    if (runTime <= time) {
      cancel();
      return;
    }
  }
};