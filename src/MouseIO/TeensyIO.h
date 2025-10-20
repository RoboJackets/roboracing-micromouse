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
  WorldCoord w = WorldCoord{};
  double lastLeftPosition = 0;
  double lastRightPosition = 0;
  double leftPosition = 0;
  double rightPosition = 0;
  double gyroYaw = 0;
  std::vector<IRSensor> sensors{IRSensor{{}, EMIT_1, RECV_1}};
  std::vector<WorldCoord> readings{};

  GridCoord getGridCoord() override { return GridCoord{x, y}; }
  WorldCoord getWorldCoord() override { return w; }
  void updateWorldCoord() override {
    double deltaLeft = getDrivePosLeft() - lastLeftPosition;
    double deltaRight = getDrivePosRight() - lastRightPosition;
    double wheelDelta = WHEEL_RADIUS_M * ((deltaLeft + deltaRight) / 2);

    double deltaX = wheelDelta * std::cos(getGyroYaw());
    double deltaY = wheelDelta * std::sin(getGyroYaw());

    w = WorldCoord{w.x + deltaX, w.y + deltaY, getGyroYaw()};
  }
  void updateEncoders() {
    // set leftpos and right pos and gyro
  }
  unsigned char getGridDir() override { return dir; }

  // void drive(double left, double right) override = 0;

  double getDriveSpeedLeft() override {
    return (getDrivePosLeft() - lastLeftPosition) / dt;
  };
  double getDriveSpeedRight() override {
    return (getDrivePosRight() - lastRightPosition) / dt;
  };

  double getDrivePosLeft() override { return leftPosition; };
  double getDrivePosRight() override { return rightPosition; };
  double getGyroYaw() override { return gyroYaw; };

  std::vector<WorldCoord> getSensorState() override { return readings; };

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
    updateEncoders();
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