#include "StartupAction.h"
#include <Arduino.h>

void StartupAction::run(MouseState &s, MouseIO &io) {
  io.setGyroOffset(io.getGyroYaw() - M_PI / 2.0);
  io.setWorldCoord(WorldCoord{0.09, ROBOT_LENGTH / 2});
  s.x = 0;
  s.y = 0;
  s.dir = TOP;
  canceled = true;
}

void StartupAction::end(MouseState &s, MouseIO &io) { canceled = false; }

DelayAction::DelayAction(double runTime) : runTime(runTime) {}

void DelayAction::run(MouseState &s, MouseIO &io) {
  if (!digitalRead(B_FRONT) || !digitalRead(B_BACK)) {
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

void DelayAction::end(MouseState &s, MouseIO &io) {
  time = 0;
  canceled = false;
  go = false;
}
