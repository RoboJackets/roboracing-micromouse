#include "StartupAction.h"

void StartupAction::run(Robot &r) {
  r.setGyroOffset(r.getGyroYaw() - M_PI / 2.0);
  r.setWorldCoord(WorldCoord{CELL_SIZE_METERS / 2.0, ROBOT_LENGTH / 2});
  canceled = true;
}

void StartupAction::end(Robot &r) { canceled = false; }

DelayAction::DelayAction(double runTime) : runTime(runTime) {}

void DelayAction::run(Robot &r) {
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

void DelayAction::end(Robot &r) {
  time = 0;
  canceled = false;
  go = false;
}
