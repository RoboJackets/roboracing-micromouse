#pragma once
#include <Arduino.h>

#include "Constants.h"
#include "IRSensor.h"
#include "IdealState.h"
#include "Mouse.h"
#include "MouseIO.h"
#include "Pins.h"
#include "Types.h"
struct TeensyIO : MouseIO {
  int x = 0;
  int y = 0;
  unsigned char dir = TOP;
  uint32_t lastMicros = 0;
  double dt = 0;
  WorldCoord w = WorldCoord{0.0, 0.0, 0.0};
  GridCoord getGridCoord() override { return GridCoord{x, y}; }
  WorldCoord getWorldCoord() override { return w; }
  void updateWorldCoord() override {}
  unsigned char getGridDir() override { return dir; }
  std::vector<IRSensor> sensors{};
  std::vector<WorldCoord> readings{};

  // void drive(double left, double right) override = 0;

  // double getDriveSpeedLeft() override = 0;
  // double getDriveSpeedRight() override = 0;

  // double getDrivePosLeft() override = 0;
  // double getDrivePosRight() override = 0;
  // double getGyroYaw() override = 0;

  std::vector<WorldCoord> getSensorState() override {
    return readings;
  };

  void updateSensorState() {
    readings.clear();
    for (int i = 0; i < sensors.size(); i++) {
      double reading = 0;
      IRSensor sensor = sensors.at(i);
      digitalWrite(sensor.EMIT, HIGH);
      delayMicroseconds(EMIT_RECV_DELAY_US);
      int post = analogRead(sensor.RECV);
      digitalWrite(sensor.EMIT, LOW);
      double dist = 0.647426 / pow(max(post, 1), 0.516999);
      readings.push_back(sensor.getReading(dist));
    }
  }

  void update(MouseState& mouseState) override {
    updateSensorState();
    uint32_t deltaMicros = micros() - lastMicros;
    dt = deltaMicros * 1e-6;
    updateWorldCoord();
  }

  void init() override {
    lastMicros = micros();
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