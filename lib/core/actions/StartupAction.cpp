#include "StartupAction.h"

void StartupAction::run(MazeMap &map, Robot &r) {
  r.setGyroOffset(r.getGyroYaw() - M_PI / 2.0);
  r.setWorldCoord(WorldCoord{CELL_SIZE_METERS / 2.0, ROBOT_LENGTH / 2});
  canceled = true;
}

void StartupAction::end(MazeMap &map, Robot &r) { canceled = false; }

DelayAction::DelayAction(double runTime) : runTime(runTime) {}

void DelayAction::run(MazeMap &map, Robot &r) {
  if (r.buttonPressed()) {
    go = true;
  }
  if (go) {
    time += r.getDt();
  }
  if (runTime <= time) {
    cancel();
    return;
  }
}

void DelayAction::end(MazeMap &map, Robot &r) {
  time = 0;
  canceled = false;
  go = false;
}
