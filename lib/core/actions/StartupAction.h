#pragma once
#include "Action.h"
#include "Constants.h"
#include "ControlAlgorithms.h"
#include "IRSensor.h"
#include "Pins.h"

struct StartupAction : Action {
  IRSensor left;
  IRSensor right;
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  PID p = PID{IRadjust};

  void run(MouseState &s, MouseIO &io) override;
  void end(MouseState &s, MouseIO &io) override;
};

struct DelayAction : Action {
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  double time = 0;
  double runTime;
  bool go = false;

  DelayAction(double runTime);

  void run(MouseState &s, MouseIO &io) override;
  void end(MouseState &s, MouseIO &io) override;
};
