#pragma once
#include <Arduino.h>

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
  std::vector<IRSensor> sensors{
      IRSensor{{}, EMIT_1, RECV_1}, IRSensor{{}, EMIT_1, RECV_1},
      IRSensor{{}, EMIT_1, RECV_1}, IRSensor{{}, EMIT_1, RECV_1}};
  std::vector<EncoderSensor> encoders{EncoderSensor{ACODER_a, ACODER_b, 0},
                                      EncoderSensor{BCODER_a, BCODER_b, 0}};
  std::vector<WorldCoord> readings{};
  std::vector<WorldCoord> readingsAverage{};

  static TeensyIO *instance;
  static void isr0() { instance->encoders[0].updateEncoder(); }
  static void isr1() { instance->encoders[1].updateEncoder(); }
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
  // IR stuff under here

  WorldCoord deriveLocalOffsetFromIR() {
    double sumX = 0;
    double sumY = 0;
    double sumTheta = 0;
    int count = 0;
    for (int i = 0; i < readings.size(); i++) {
      WorldCoord reading = readings[i];
      if (std::isfinite(reading.hypot()) && reading.hypot() > 0 &&
          reading.hypot() < CELL_SIZE_METERS) {
        sumX += reading.x;
        sumY += reading.y;
        sumTheta += reading.theta;
        count++;
      }
    }
    return WorldCoord{sumX / count, sumY / count, sumTheta / count};
  }

  WorldCoord correct() {
    WorldCoord localError = deriveLocalOffsetFromIR();

    double correctedLocalX = localError.x * 0.1 * std::exp(-std::abs(localError.x));
    double correctedLocalY = localError.y * 0.1 * std::exp(-std::abs(localError.y));

    // Now rotate the independently weighted components
    double globalDeltaX = correctedLocalX * std::cos(w.theta) - correctedLocalY * std::sin(w.theta);
    double globalDeltaY = correctedLocalX * std::sin(w.theta) + correctedLocalY * std::cos(w.theta);

    return {w.x + globalDeltaX, w.y + globalDeltaY, w.theta};
}

  WorldCoord getWorldCoord() override { return w; }
  void updateWorldCoord() override {
    double deltaLeft = getDrivePosLeft() - lastLeftPosition;
    double deltaRight = getDrivePosRight() - lastRightPosition;
    double wheelDelta = ((deltaLeft + deltaRight) / 2);

    if (readings.size() >= 2 && readings.at(0).hypot() < 0.18 &&
        readings.at(1).hypot() < 0.18) {
      double deltaR = readingsAverage.at(0).y - readingsAverage.at(1).y;
      double sensorYaw = std::atan2(deltaR, FRONT_SENSOR_SEP);
      double currentHeading = gyroYaw - gyroOffset;
      double nearestCardinal =
          std::round(currentHeading / (M_PI / 2.0)) * (M_PI / 2.0);
      double sensorOffset = gyroYaw - nearestCardinal - sensorYaw;
      gyroOffset = GYRO_ALPHA * gyroOffset + (1.0 - GYRO_ALPHA) * sensorOffset;
    }
    double theta = getGyroYaw() - gyroOffset;
    double deltaX = wheelDelta * std::cos(theta);
    double deltaY = wheelDelta * std::sin(theta);

    w = WorldCoord{w.x + deltaX, w.y + deltaY, theta};
    // IR correction here
    w = correct();
  }
  void updateEncoders() {
    lastLeftPosition = leftPosition;
    lastRightPosition = rightPosition;
    leftPosition = encoders[0].getPosition();
    rightPosition = encoders[1].getPosition();
  }
  unsigned char getGridDir() override { return dir; }

  void driveVoltage(double left, double right) override {
    double l = std::clamp(left, -1.0, 1.0);
    double r = std::clamp(right, -1.0, 1.0);
    mA.drive((int)(r * 255));
    mB.drive((int)(l * 255));
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

  std::vector<WorldCoord> getSensorState() override { return readings; };
  std::vector<WorldCoord> getAverageSensorState() override {
    return readingsAverage;
  };
  void updateDt() {
    uint32_t now = micros();
    uint32_t deltaMicros = now - lastMicros;
    lastMicros = now;
    cachedDt = deltaMicros * 1e-6;
  }

  double getDt() override { return cachedDt; }

  void updateSensorState() {
    readings.clear();
    readingsAverage.clear();
    for (int i = 0; i < sensors.size(); i++) {
      IRSensor sensor = sensors.at(i);
      digitalWrite(sensor.EMIT, HIGH);
      delayMicroseconds(EMIT_RECV_DELAY_US);
      int post = analogRead(sensor.RECV);
      digitalWrite(sensor.EMIT, LOW);
      // relative to mouse in m
      double dist = post < 4 ? std::numeric_limits<double>::infinity()
                             : 0.647426 / pow(max(post, 1), 0.516999);
      readings.push_back(sensor.getReading(dist));
      readingsAverage.push_back(sensor.getAverage());
    }
    gyro.update();
    gyroYaw = gyro.ypr[0];
    // Serial.println(readings.at(0).hypot());
  }

  void updateMazeState(MouseState &mouseState) {
    WorldCoord gridRelative = w.gridRelativeCoords();
    int gx = getGridCoord().x;
    int gy = getGridCoord().y;
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
        unsigned char oppositeWall =
            ((sensedWall >> 2) | (sensedWall << 2)) & 0x0F;
        mouseState.walls[gx + cx][gy + cy] |= oppositeWall;
      }
    }
  }

  void update(MouseState &mouseState) override {
    updateDt();
    updateSensorState();
    updateEncoders();
    updateWorldCoord();
    updateMazeState(mouseState);
    Logger::tick();
  }

  void init() override {
    instance = this;
    Logger::init();
    lastMicros = micros();
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(EMIT_1, OUTPUT);
    pinMode(ACODER_a, INPUT);
    pinMode(ACODER_b, INPUT);
    attachInterrupt(digitalPinToInterrupt(ACODER_a), isr0, CHANGE);
    pinMode(BCODER_a, INPUT);
    pinMode(BCODER_b, INPUT);
    attachInterrupt(digitalPinToInterrupt(BCODER_a), isr1, CHANGE);
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

TeensyIO *TeensyIO::instance = nullptr;