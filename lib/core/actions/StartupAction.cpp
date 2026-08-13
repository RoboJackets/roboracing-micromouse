#include "StartupAction.h"

void StartupAction::run(MazeMap &map, MouseIO &io) {
  io.setGyroOffset(io.getGyroYaw() - M_PI / 2.0);
  io.setWorldCoord(WorldCoord{CELL_SIZE_METERS / 2.0, ROBOT_LENGTH / 2});
  canceled = true;
}

void StartupAction::end(MazeMap &map, MouseIO &io) { canceled = false; }

DelayAction::DelayAction(double runTime) : runTime(runTime) {}

void DelayAction::run(MazeMap &map, MouseIO &io) {
  if (io.buttonPressed()) {
    go = true;
  }
  if (go) {
    time += io.getDt();
  }
  if (runTime <= time) {
    cancel();
    return;
  }
}

void DelayAction::end(MazeMap &map, MouseIO &io) {
  time = 0;
  canceled = false;
  go = false;
}
