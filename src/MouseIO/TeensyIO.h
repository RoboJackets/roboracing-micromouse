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
#include "Types.h"
#include <DRV8833.h>
#include <Gyro.cpp>

struct TeensyIO : MouseIO {
  static TeensyIO *instance;

  unsigned char dir = TOP;
  uint32_t lastMicros = 0;
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
    int gx = (int)(w.x / CELL_SIZE_METERS + 0.5);
    int gy = (int)(w.y / CELL_SIZE_METERS + 0.5);
    unsigned char theta = getGyroYaw();
    // TODO: update gyro
    return GridCoord{gx, gy};
  }

  WorldCoord getWorldCoord() override { return w; }
  void updateWorldCoord() override {
    double deltaLeft = getDrivePosLeft() - lastLeftPosition;
    double deltaRight = getDrivePosRight() - lastRightPosition;
    double wheelDelta = ((deltaLeft + deltaRight) / 2);

    if (readings.size() >= 2 && readings.at(0).hypot() < 0.18 &&
        readings.at(1).hypot() < 0.18) {
      double deltaR = readings.at(0).y - readings.at(1).y;
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
      double dist = post < 4 ? std::numeric_limits<double>::infinity()
                             : 0.647426 / pow(max(post, 1), 0.516999);
      Serial.println(post);
      readings.push_back(sensor.getReading(dist));
    }
    gyro.update();
    gyroYaw = gyro.ypr[0];
    // Serial.println(readings.at(0).hypot());
  }

  void update(MouseState &mouseState) override {
    updateSensorState();
    updateEncoders();
    updateWorldCoord();
  }

  void init() override {
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