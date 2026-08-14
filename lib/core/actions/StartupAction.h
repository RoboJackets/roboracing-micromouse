#pragma once
#include "Action.h"
#include "Constants.h"

struct StartupAction : Action {

  void run(Robot &r) override;
};

struct DelayAction : Action {
  double time = 0;
  double runTime;
  bool go = false;

  DelayAction(double runTime);

  void run(Robot &r) override;
};
