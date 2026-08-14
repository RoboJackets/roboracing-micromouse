#pragma once
#include "Action.h"
#include "Constants.h"

struct StartupAction : Action {
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  void run(MazeMap &map, Robot &r) override;
  void end(MazeMap &map, Robot &r) override;
};

struct DelayAction : Action {
  bool canceled = false;
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }
  double time = 0;
  double runTime;
  bool go = false;

  DelayAction(double runTime);

  void run(MazeMap &map, Robot &r) override;
  void end(MazeMap &map, Robot &r) override;
};
