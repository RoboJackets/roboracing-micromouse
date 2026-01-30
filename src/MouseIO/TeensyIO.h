#pragma once
#include <Arduino.h>

#include <cmath>

#include "Constants.h"
#include "IRSensor.h"
#include "IdealState.h"
#include "Mouse.h"
#include "MouseIO.h"
#include "Pins.h"
#include "Types.h"
#include <Gyro.cpp>
#include <SparkFun_TB6612.cpp>

struct TeensyIO : MouseIO {
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

  Gyro gyro{};

  Motor mA = Motor(AIN1, AIN2, PWMA, 1, STBY);
  Motor mB = Motor(BIN1, BIN2, PWMB, 1, STBY);

  GridCoord getGridCoord() override {
    int gx = (int)(w.x / CELL_SIZE_METERS + 0.5);
    int gy = (int)(w.y / CELL_SIZE_METERS + 0.5);
    return GridCoord{gx, gy};
  }

  WorldCoord getWorldCoord() override { return w; }
  void updateWorldCoord() override {
    double deltaLeft = getDrivePosLeft() - lastLeftPosition;
    double deltaRight = getDrivePosRight() - lastRightPosition;
    double wheelDelta = 2 * M_PI * WHEEL_RADIUS_M / COUNTS_PER_REVOLUTION *
                        ((deltaLeft + deltaRight) / 2);

    double deltaX = wheelDelta * std::cos(getGyroYaw());
    double deltaY = wheelDelta * std::sin(getGyroYaw());

    w = WorldCoord{w.x + deltaX, w.y + deltaY, getGyroYaw()};
  }
  void updateEncoders() {
    // set leftpos and right pos and gyro
  }
  unsigned char getGridDir() override { return dir; }

  void drive(double left, double right) override {
    double l = std::clamp(left, -1.0, 1.0);
    double r = std::clamp(right, -1.0, 1.0);
    mA.drive(l * 255);
    mB.drive(r * 255);
  }

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

  double getDt() override { return dt; }

  void updateSensorState() {
    readings.clear();
    for (int i = 0; i < sensors.size(); i++) {
      IRSensor sensor = sensors.at(i);
      digitalWrite(sensor.EMIT, HIGH);
      delayMicroseconds(EMIT_RECV_DELAY_US);
      int post = analogRead(sensor.RECV);
      digitalWrite(sensor.EMIT, LOW);
      double dist = post < 4 ? std::numeric_limits<double>::infinity()
                             : 0.647426 / pow(max(post, 1), 0.516999);
      readings.push_back(sensor.getReading(dist));
      // std::cout << "V: " << std::to_string(post)
      //           << "x: " << std::to_string(sensor.getReading(dist).x)
      //           << " y: " << std::to_string(sensor.getReading(dist).y)
      //           << std::endl;
    }
    gyro.update();
    gyroYaw = gyro.ypr[0] * 180.0 / M_PI;
    Serial.println(gyroYaw);
  }

  void update(MouseState &mouseState) override {
    uint32_t now = micros();
    uint32_t deltaMicros = now - lastMicros;
    lastMicros = now;
    dt = deltaMicros * 1e-6;
    updateSensorState();
    updateEncoders();
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

    gyro.initalizeGyro();

    Serial.begin(9600);
  };
};