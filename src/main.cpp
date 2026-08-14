#include <Arduino.h>

#include "StateMachine.h"
#include "TeensyIO.h"
#include "robot/Robot.h"

TeensyIO teensyIO{};
Robot robot{teensyIO};
StateMachine mouse{};

void setup() { mouse.init(robot); }

void loop() {
  mouse.tick(robot);

  const WorldCoord w = robot.getWorldCoord();
  Serial.printf("WORLD: %0.2f, %0.2f  THETA: %0.2f\n", w.x, w.y, w.theta);
  const auto s = robot.getSensorState();
  for (int i = 0; i < 4; i++) {
    Serial.printf("%d: %0.2f, %0.2f     ", i, s[i].x, s[i].y);
  }
}
