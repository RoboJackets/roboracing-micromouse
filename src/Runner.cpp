#include <Arduino.h>

#include "MouseIO.h"
#include "StateMachine.h"
#include "TeensyIO.h"

MouseIO *io = nullptr;
TeensyIO teensyIO = TeensyIO{};

void setup() {
  io = &teensyIO;
  StateMachine::init(io);
}

void loop() { StateMachine::tick(io); }
