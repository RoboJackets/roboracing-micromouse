#include <Arduino.h>

#include "StateMachine.h"
#include "TeensyIO.h"

TeensyIO teensyIO{};
StateMachine mouse{};

void setup() { mouse.init(teensyIO); }

void loop() { mouse.tick(teensyIO); }
