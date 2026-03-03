#pragma once
#include <Arduino.h>

#include <cmath>

#include "Constants.h"
#include "ControlAlgorithms.h"
#include "IRSensor.h"
#include "IdealState.h"
#include "Mouse.h"
#include "MouseIO.h"
#include "Pins.h"
#include "Types.h"
#include <DRV8833.h>
#include <Gyro.cpp>

struct TeensyIO : MouseIO {
  unsigned char dir = TOP;
  uint32_t lastMicros = 0;
  WorldCoord w = WorldCoord{};
  double lastLeftPosition = 0;
  double lastRightPosition = 0;
  double leftPosition = 0;
  double rightPosition = 0;
  double gyroYaw = 0;
  double width; // left to right
  double length; // front to back
  std::vector<IRSensor> sensors{IRSensor{{}, EMIT_1, RECV_1}};
  std::vector<WorldCoord> readings{};

  Gyro gyro{};
  PID velocityPIDRight{velocityPIDConstants};
  PID velocityPIDLeft{velocityPIDConstants};

  MotorFeedForward leftff{0, 0, 0};
  MotorFeedForward rightff{0, 0, 0};

  DRV8833Motor mA = DRV8833Motor(AIN1, AIN2, -1, STBY);
  DRV8833Motor mB = DRV8833Motor(BIN1, BIN2, 1, STBY);

  GridCoord getGridCoord() override {
    int gx = (int)(w.x / CELL_SIZE_METERS + 0.5);
    int gy = (int)(w.y / CELL_SIZE_METERS + 0.5);
    double angle = w.theta;
    unsigned char dir = (angle >= 315 || angle < 45) 
                      || (1 << (angle >= 45 && angle < 135))
                      || (2 << (angle >= 135 && angle < 225))
                      || (3 << (angle >= 225 && angle < 315));
    return GridCoord{gx, gy, dir};
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

  void driveVoltage(double left, double right) override {
    double l = std::clamp(left, -1.0, 1.0);
    double r = std::clamp(right, -1.0, 1.0);
    mA.drive((int)(r * 255));
    mB.drive((int)(l * 255));
  }

  void driveVelocity(double left, double right) override {
    driveVoltage(
        leftff.calculate(left, getDt()) +
            velocityPIDLeft.calculate(getDriveSpeedLeft(), left, getDt()),
        rightff.calculate(right, getDt()) +
            velocityPIDRight.calculate(getDriveSpeedRight(), right, getDt()));
  }

  double getDriveSpeedLeft() override {
    return (getDrivePosLeft() - lastLeftPosition) / getDt();
  };
  double getDriveSpeedRight() override {
    return (getDrivePosRight() - lastRightPosition) / getDt();
  };

  double getDrivePosLeft() override { return leftPosition; };
  double getDrivePosRight() override { return rightPosition; };
  double getGyroYaw() override { return gyroYaw; };

  std::vector<WorldCoord> getSensorState() override { return readings; };

  double getDt() override {
    uint32_t now = micros();
    uint32_t deltaMicros = now - lastMicros;
    lastMicros = now;
    double dt = deltaMicros * 1e-6;
    return dt;
  }

  void updateSensorState() {
    readings.clear();
    for (int i = 0; i < sensors.size(); i++) {
      IRSensor sensor = sensors.at(i);
      digitalWrite(sensor.EMIT, HIGH);
      delayMicroseconds(EMIT_RECV_DELAY_US);
      int post = analogRead(sensor.RECV);
      digitalWrite(sensor.EMIT, LOW);
      // relative to mouse in m
      double dist = post < 4 ? std::numeric_limits<double>::infinity()
                             : 0.647426 / pow(max(post, 1), 0.516999);
      Serial.println(post);
      readings.push_back(sensor.getReading(dist));
    }
    gyro.update();
    gyroYaw = gyro.ypr[0] * 180.0 / M_PI;
    // Serial.println(readings.at(0).hypot());
  }

  void updateMazeState(MouseState &mouseState) {
    // 18 x 18 cm squares
    int cell_size_cm = (int)(CELL_SIZE_METERS * 100);
    double square_x = ((int)(w.x * 100) % cell_size_cm) / 100;
    double square_y = ((int)(w.y * 100) % cell_size_cm) / 100;
    int gx = getGridCoord().x;
    int gy = getGridCoord().y;
    for (int i = 0; i < readings.size(); i++) {
        if (square_x + readings.at(i).x < CELL_SIZE_METERS && square_y + readings.at(i).y < CELL_SIZE_METERS) {
          double angle = w.theta;
          // left 45
          if (i == 2) {
            angle += 45;
          // right 45
          } else if (i == 3) {
            angle -= 45;
          }
          mouseState.walls[gx][gy] |= 3 << (angle >= 315 || angle < 45);
          if (gx + 1 < N) {
            mouseState.walls[gx+1][gy] |= 3 << (angle >= 315 || angle < 45);
          }
          mouseState.walls[gx][gy] |= (angle >= 45 || angle < 135);
          if (gy + 1 < N) {
            mouseState.walls[gx][gy+1] |= (angle >= 45 || angle < 135);
          }
          mouseState.walls[gx][gy] |= 2 << (angle >= 135 || angle < 225);
          if (gx - 1 >= 0) {
            mouseState.walls[gx-1][gy] |= 2 << (angle >= 135 || angle < 225);
          }
          mouseState.walls[gx][gy] |= 1 << (angle >= 225 || angle < 315);
          if (gy - 1 >= 0) {
            mouseState.walls[gx][gy-1] |= 1 << (angle >= 225 || angle < 315);
          }
      } 
    }
    
  }

  void update(MouseState &mouseState) override {
    updateSensorState();
    updateMazeState(mouseState);
    // updateEncoders();
    // updateWorldCoord();
  }

  void init() override {
    lastMicros = micros();
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(EMIT_1, OUTPUT);
    // pinMode(EMIT_2, OUTPUT);
    // pinMode(EMIT_3, OUTPUT);
    // pinMode(EMIT_4, OUTPUT);

    pinMode(RECV_1, INPUT);
    // pinMode(RECV_2, INPUT);
    // pinMode(RECV_3, INPUT);
    // pinMode(RECV_4, INPUT);

    gyro.initalizeGyro();

    Serial.begin(9600);
  };
};