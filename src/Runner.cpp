#include "MouseIO.h"
#include "StateMachine.h"
MouseIO* io = nullptr;

#ifdef MMS_SIM
#include "MMSIO.h"
#include "../mms-cpp/API.cpp"
MMSIO mms = MMSIO{};
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
TeensyIO teensyIO = TeensyIO{};
void setup() {
  io = &teensyIO;
  StateMachine::init(io);
}
void loop() { StateMachine::tick(io); }
#endif