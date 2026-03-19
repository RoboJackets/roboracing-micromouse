#pragma once
#include <Arduino.h>

#include <array>
#include <cmath>

#include "Constants.h"
#include "ControlAlgorithms.h"
#include "EncoderSensor.h"
#include "IRSensor.h"
#include "IdealState.h"
#include "Mouse.h"
#include "MouseIO.h"
#include "Pins.h"
#include "SDLogger.h"
#include "Types.h"
#include <DRV8833.h>
#include <Gyro.cpp>

struct TeensyIO : MouseIO {
  unsigned char dir = TOP;
  uint32_t lastMicros = 0;
  double cachedDt = 0;
  WorldCoord w = WorldCoord{};
  double lastLeftPosition = 0;
  double lastRightPosition = 0;
  double leftPosition = 0;
  double rightPosition = 0;
  double gyroYaw = 0;
  // FL, FR, DL, DR
  std::vector<IRSensor> sensors{
      IRSensor{{}, EMIT_1, RECV_1}, IRSensor{{}, EMIT_4, RECV_4},
      IRSensor{{}, EMIT_2, RECV_2}, IRSensor{{}, EMIT_3, RECV_3}};
  EncoderSensor encoderLeft{ACODER_a, ACODER_b, 0, true};
  EncoderSensor encoderRight{BCODER_a, BCODER_b, 0, false};
  std::array<WorldCoord, 4> readings{};
  std::array<WorldCoord, 4> readingsAverage{};
  double gyroOffset = 0;

  Gyro gyro{};
  PID velocityPIDRight{velocityPIDConstants};
  PID velocityPIDLeft{velocityPIDConstants};

  MotorFeedForward leftff{0, 0, 0};
  MotorFeedForward rightff{0, 0, 0};

  DRV8833Motor mA = DRV8833Motor(AIN1, AIN2, -1, STBY);
  DRV8833Motor mB = DRV8833Motor(BIN1, BIN2, 1, STBY);

  GridCoord getGridCoord() override {
    int gx = std::lround(w.x / CELL_SIZE_METERS);
    int gy = std::lround(w.y / CELL_SIZE_METERS);
    unsigned char dir = getGridDir(w.theta);
    // TODO: update gyro
    return GridCoord{gx, gy, dir};
  }

  unsigned char getGridDir(double angle) {
    double deg = std::fmod(angle * 180.0 / M_PI, 360.0);
    if (deg < 0)
      deg += 360;
    if (deg >= 315 || deg < 45)
      return TOP;
    if (deg < 135)
      return RIGHT;
    if (deg < 225)
      return DOWN;
    return LEFT;
  }

  WorldCoord getWorldCoord() override { return w; }
  void updateWorldCoord() override {
    double deltaLeft = getDrivePosLeft() - lastLeftPosition;
    double deltaRight = getDrivePosRight() - lastRightPosition;
    double wheelDelta = ((deltaLeft + deltaRight) / 2);

    // if (readings.size() >= 2 && readings.at(0).hypot() < 0.18 &&
    //     readings.at(1).hypot() < 0.18) {
    //   double deltaR = readingsAverage.at(0).y - readingsAverage.at(1).y;
    //   double sensorYaw = std::atan2(deltaR, FRONT_SENSOR_SEP);
    //   double currentHeading = gyroYaw - gyroOffset;
    //   double nearestCardinal =
    //       std::round(currentHeading / (M_PI / 2.0)) * (M_PI / 2.0);
    //   double sensorOffset = gyroYaw - nearestCardinal - sensorYaw;
    //   gyroOffset = GYRO_ALPHA * gyroOffset + (1.0 - GYRO_ALPHA) *
    //   sensorOffset;
    // }
    double theta = getGyroYaw() - gyroOffset;
    double deltaX = wheelDelta * std::cos(theta);
    double deltaY = wheelDelta * std::sin(theta);

    w = WorldCoord{w.x + deltaX, w.y + deltaY, theta};
  }
  void updateEncoders() {
    lastLeftPosition = leftPosition;
    lastRightPosition = rightPosition;
    leftPosition = encoderLeft.getPosition();
    rightPosition = encoderRight.getPosition();
  }
  unsigned char getGridDir() override { return dir; }

  void driveVoltage(double left, double right) override {
    double l = std::clamp(left, -1.0, 1.0);
    double r = std::clamp(right, -1.0, 1.0);
    // Serial.println(l);
    mA.drive((int)(l * 255));
    mB.drive((int)(r * 255));
  }
  void setGyroOffset(double offset) { gyroOffset = offset; }

  void setWorldCoord(WorldCoord c) { w = c; }

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

  std::array<WorldCoord, 4> getSensorState() override { return readings; };
  std::array<WorldCoord, 4> getAverageSensorState() override {
    return readingsAverage;
  };
  void updateDt() {
    uint32_t now = micros();
    uint32_t deltaMicros = now - lastMicros;
    lastMicros = now;
    cachedDt = std::max(deltaMicros * 1e-6, 1e-6);
  }

  double getDt() override { return cachedDt; }

  void updateSensorState() {
    for (int i = 0; i < sensors.size(); i++) {
      IRSensor sensor = sensors.at(i);
      digitalWrite(sensor.EMIT, HIGH);
      delayMicroseconds(EMIT_RECV_DELAY_US);
      int post = analogRead(sensor.RECV);
      digitalWrite(sensor.EMIT, LOW);
      // relative to mouse in m
      double dist = post < 4 ? std::numeric_limits<double>::infinity()
                             : 0.647426 / pow(max(post, 1), 0.516999);
      readings[i] = sensor.getReading(dist);
      readingsAverage[i] = sensor.getAverage();
      // Serial.print(i);
      // Serial.print(": ");
      // Serial.print(post);
      // Serial.print("     ");
    }
    // Serial.print("GYRO: ");
    // Serial.print(gyroYaw);
    // Serial.print("    ");
    // Serial.println(leftPosition);
    gyro.update();
    gyroYaw = gyro.ypr[0];
  }

  void updateMazeState(MouseState &mouseState) {
    WorldCoord gridRelative = w.gridRelativeCoords();
    int gx = getGridCoord().x;
    int gy = getGridCoord().y;
    if (gx < 0 || gx >= N || gy < 0 || gy >= N)
      return;
    for (int i = 0; i < sensors.size(); i++) {
      if (gridRelative.x + readings.at(i).x < CELL_SIZE_METERS &&
          gridRelative.y + readings.at(i).y < CELL_SIZE_METERS) {
        double angle = gridRelative.theta + sensors.at(i).pos_from_center.theta;
        unsigned char sensedWall = getGridDir(angle);
        mouseState.walls[gx][gy] |= sensedWall;
        int cx = ((std::abs(std::cos(angle)) > sqrt(2) / 2) ? 1 : 0) *
                 (std::cos(angle) < 0 ? -1 : 1);
        int cy = ((std::abs(std::sin(angle)) > sqrt(2) / 2) ? 1 : 0) *
                 (std::sin(angle) < 0 ? -1 : 1);
        int nx = gx + cx;
        int ny = gy + cy;
        if (nx >= 0 && nx < N && ny >= 0 && ny < N) {
          unsigned char oppositeWall =
              ((sensedWall >> 2) | (sensedWall << 2)) & 0x0F;
          mouseState.walls[nx][ny] |= oppositeWall;
        }
      }
    }
  }

  void update(MouseState &mouseState) override {
    updateDt();
    updateSensorState();
    updateEncoders();
    updateWorldCoord();
    updateMazeState(mouseState);
    driveVoltage(-0.4, 0);
    // Logger::tick();
  }

  void init() override {
    // Logger::init();
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

    mA.begin();
    mB.begin();

    Serial.begin(9600);
  };
};