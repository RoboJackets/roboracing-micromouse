#pragma once
#include <Arduino.h>

#include "Constants.h"
#include "IdealState.h"
#include "Mouse.h"
#include "MouseIO.h"
#include "Pins.h"
#include "Types.h"
struct TeensyIO : MouseIO {
  int x = 0;
  int y = 0;
  unsigned char dir = TOP;
  WorldCoord w = WorldCoord{0.0, 0.0, 0.0};
  GridCoord getGridCoord() override { return GridCoord{x, y}; }
  WorldCoord getWorldCoord() override { return w; }
  void updateWorldCoord() override {}
  unsigned char getGridDir() override { return dir; }

  void drive(double left, double right) override = 0;

  double getDriveSpeedLeft() override = 0;
  double getDriveSpeedRight() override = 0;

  double getDrivePosLeft() override = 0;
  double getDrivePosRight() override = 0;
  double getGyroYaw() override = 0;

  std::vector<WorldCoord> getSensorState() override = 0;

  void update(MouseState& mouseState) override = 0;

  void init() override {
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(EMIT_1, OUTPUT);
    pinMode(EMIT_2, OUTPUT);
    pinMode(EMIT_3, OUTPUT);
    pinMode(EMIT_4, OUTPUT);

    pinMode(RECV_1, INPUT);
    pinMode(RECV_2, INPUT);
    pinMode(RECV_3, INPUT);
    pinMode(RECV_4, INPUT);

    Serial.begin(9600);
  };
};