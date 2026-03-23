#pragma once
#include "Action.h"
#include "ControlAlgorithms.h"
#include <Constants.h>
#include <IRSensor.h>
#include <Pins.h>

struct StartupAction : Action {
  IRSensor left;
  IRSensor right;
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  PID p = PID{IRadjust};
  void run(MouseState &s, MouseIO &io) override {
    io.setGyroOffset(io.getGyroYaw() - M_PI / 2.0);
    io.setWorldCoord(WorldCoord{0.09, ROBOT_LENGTH / 2});
    s.x = 0;
    s.y = 0;
    s.dir = TOP;
    canceled = true;
  }
  void end(MouseState &s, MouseIO &io) override { canceled = false; }
};
struct DelayAction : Action {
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  double time = 0;
  double runTime;
  bool go = false;
  DelayAction(double runTime) : runTime(runTime) {}
  void run(MouseState &s, MouseIO &io) override {
    if (!digitalRead(B_FRONT) || !digitalRead(B_BACK)) {
      go = true;
    }
    if (go) {
      time += io.getDt();
    }
    // Serial.println(time);
    if (runTime <= time) {
      cancel();
      return;
    }
  }
  void end(MouseState &s, MouseIO &io) override {
    time = 0;
    canceled = false;
    go = false;
  }
};