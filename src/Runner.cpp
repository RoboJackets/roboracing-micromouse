#include "MMSIO.h"
#include "MouseIO.h"
#include "StateMachine.h"

MouseIO* io = nullptr;
MMSIO mms = MMSIO{};
#ifdef MMS_SIM
int main() {
  io = &mms;
  StateMachine::init(io);
  while (true) {
    StateMachine::tick(io);
  }
}
#else
#include <Arduino.h>

#include "TeensyIO.h"
TeensyIO teensy = TeensyIO{};
void setup() { StateMachine::init(io); }
void loop() { StateMachine::tick(io); }
#endif