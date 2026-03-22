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
  bool mazeUpdate = false;
  // FL, FR, DL, DR
  std::vector<IRSensor> sensors{
      IRSensor{{-0.0473, 0.013, PI / 2}, EMIT_1, RECV_1, 0.791794, 0.451947},
      IRSensor{{0.0473, 0.013, PI / 2}, EMIT_4, RECV_4, 0.791794, 0.451947},
      IRSensor{{-0.021, 0.038, PI / 2 + 30 * (PI / 180)},
               EMIT_2,
               RECV_2,
               0.791794,
               0.451947},
      IRSensor{{0.021, 0.038, PI / 2 - 30 * (PI / 180)},
               EMIT_3,
               RECV_3,
               0.791794,
               0.451947}};
  EncoderSensor encoderLeft{ACODER_a, ACODER_b, 0, true};
  EncoderSensor encoderRight{BCODER_a, BCODER_b, 0, false};
  std::array<WorldCoord, 4> readings{};
  std::array<WorldCoord, 4> readingsAverage{};
  double gyroOffset = 0;

  double filteredSpeedLeft = 0;
  double filteredSpeedRight = 0;

  Gyro gyro{};
  PID velocityPIDRight{velocityPIDConstants};
  PID velocityPIDLeft{velocityPIDConstants};

  MotorFeedForward leftff{0.45, 0.7, 0};
  MotorFeedForward rightff{0.45, 0.7, 0};

  DRV8833Motor mLeft = DRV8833Motor(AIN1, AIN2, 1, STBY);
  DRV8833Motor mRight = DRV8833Motor(BIN1, BIN2, 1, STBY);

  GridCoord getGridCoord() override {
    int gx = std::floor(w.x / CELL_SIZE_METERS);
    int gy = std::floor(w.y / CELL_SIZE_METERS);
    unsigned char dir = getGridDir(w.theta);
    // TODO: update gyro
    return GridCoord{gx, gy, dir};
  }

  void resetPIDs() override {
    velocityPIDLeft.resetAccum();
    velocityPIDRight.resetAccum();
  }

  unsigned char getGridDir(double angle) {
    double deg = std::fmod(angle * 180.0 / M_PI, 360.0);
    if (deg < 0)
      deg += 360;
    if (deg >= 315 || deg < 45)
      return RIGHT;
    if (deg < 135)
      return TOP;
    if (deg < 225)
      return LEFT;
    return DOWN;
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
    // Serial.printf("GYRO: %0.2f\n", w.theta);
    double theta = (-getGyroYaw() - gyroOffset);
    double deltaX = wheelDelta * std::cos(theta);
    double deltaY = wheelDelta * std::sin(theta);

    w = WorldCoord{w.x + deltaX, w.y + deltaY, theta};
  }
  void updateEncoders() {
    lastLeftPosition = leftPosition;
    lastRightPosition = rightPosition;
    leftPosition = encoderLeft.getPosition();
    rightPosition = encoderRight.getPosition();

    double dt = getDt();
    double rawLeft = (leftPosition - lastLeftPosition) / dt;
    double rawRight = (rightPosition - lastRightPosition) / dt;

    constexpr double alpha = 0.4;
    filteredSpeedLeft += alpha * (rawLeft - filteredSpeedLeft);
    filteredSpeedRight += alpha * (rawRight - filteredSpeedRight);
  }
  unsigned char getGridDir() override { return dir; }

  void driveVoltage(double left, double right) override {
    double l = std::clamp(left, -1.0, 1.0);
    double r = std::clamp(right, -1.0, 1.0);
    if (l == 0.0 && r == 0.0) {
      mLeft.brake();
      mRight.brake();
      Serial.println("BRAKE!!");
    } else {
      mLeft.drive((int)(l * 255));
      mRight.drive((int)(r * 255));
    }
  }
  void setGyroOffset(double offset) { gyroOffset = offset; }

  void setWorldCoord(WorldCoord c) { w = c; }

  void driveVelocity(double left, double right) override {
    // Serial.printf("LEFT ERROR: %0.2f\n", getDriveSpeedLeft() - left);
    // if (left > 0.1) {
    //   Serial.printf("LEFT: %0.2f, RIGHT: %0.2f, LA: %0.2f, RA: %0.2f\n",
    //   left,
    //                 right, getDriveSpeedLeft(), getDriveSpeedRight());
    // }
    driveVoltage(
        leftff.calculate(left, getDt()) +
            velocityPIDLeft.calculate(getDriveSpeedLeft(), left, getDt()),
        rightff.calculate(right, getDt()) +
            velocityPIDRight.calculate(getDriveSpeedRight(), right, getDt()));
  }

  double getDriveSpeedLeft() override { return filteredSpeedLeft; }
  double getDriveSpeedRight() override { return filteredSpeedRight; };

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

  void allowUpdates(bool x) { mazeUpdate = x; }

  void updateSensorState() {
    for (int i = 0; i < sensors.size(); i++) {
      IRSensor &sensor = sensors.at(i);
      digitalWrite(sensor.EMIT, HIGH);
      delayMicroseconds(EMIT_RECV_DELAY_US);
      int post = analogRead(sensor.RECV);
      digitalWrite(sensor.EMIT, LOW);
      // relative to mouse in m
      readings[i] = sensor.getReading(post);
      readingsAverage[i] = sensor.getAverage();
      Serial.print(i);
      Serial.print(": ");
      Serial.printf("%0.2f, %0.2f", readings[i].x, readings[i].y);
      Serial.print("     ");
    }
    // Serial.print("GYRO: ");
    // Serial.print(w.theta);
    // Serial.print("    ");
    // Serial.printf("X: %0.2f Y: %0.2f\n", w.x, w.y);
    gyro.update();
    gyroYaw = gyro.ypr[0];
  }

  void updateMazeState(MouseState &mouseState) {
    if (std::abs(std::remainder(w.theta, PI / 2.0)) > 0.12)
      return;
    if (!mazeUpdate)
      return;

    int gx = getGridCoord().x;
    int gy = getGridCoord().y;
    if (gx < 0 || gx >= N || gy < 0 || gy >= N)
      return;

    unsigned char dir = getGridCoord().dir;

    double cosH = std::cos(w.theta);
    double sinH = std::sin(w.theta);

    double cellLeft = gx * CELL_SIZE_METERS;
    double cellRight = (gx + 1) * CELL_SIZE_METERS;
    double cellBottom = gy * CELL_SIZE_METERS;
    double cellTop = (gy + 1) * CELL_SIZE_METERS;

    constexpr double MAX_READING_DIST = 0.17;

    // diagonal sensors: looser sideways tolerance, very strict forward
    // overshoot
    constexpr double SIDE_TOL = 0.065;
    constexpr double SIDE_FORWARD_MARGIN = 0.006;
    constexpr double SIDE_BACK_MARGIN = 0.02;

    // front sensors
    constexpr double FRONT_TOL = 0.03;
    constexpr double FRONT_SIDE_MARGIN = 0.03;

    for (int i = 0; i < (int)sensors.size(); i++) {
      WorldCoord r = readingsAverage[i];
      if (!std::isfinite(r.x) || !std::isfinite(r.y))
        continue;

      if (r.hypot() > MAX_READING_DIST)
        continue;

      double wx = w.x + r.x * sinH + r.y * cosH;
      double wy = w.y - r.x * cosH + r.y * sinH;

      unsigned char wall = 0;

      if (i == 3) { // right diagonal
        if (dir == TOP) {
          if (std::abs(wx - cellRight) < SIDE_TOL &&
              wy >= cellBottom - SIDE_BACK_MARGIN &&
              wy <= cellTop + SIDE_FORWARD_MARGIN)
            wall = RIGHT;
        } else if (dir == RIGHT) {
          if (std::abs(wy - cellBottom) < SIDE_TOL &&
              wx >= cellLeft - SIDE_BACK_MARGIN &&
              wx <= cellRight + SIDE_FORWARD_MARGIN)
            wall = DOWN;
        } else if (dir == LEFT) {
          if (std::abs(wy - cellTop) < SIDE_TOL &&
              wx >= cellLeft - SIDE_FORWARD_MARGIN &&
              wx <= cellRight + SIDE_BACK_MARGIN)
            wall = TOP;
        } else if (dir == DOWN) {
          if (std::abs(wx - cellLeft) < SIDE_TOL &&
              wy >= cellBottom - SIDE_FORWARD_MARGIN &&
              wy <= cellTop + SIDE_BACK_MARGIN)
            wall = LEFT;
        }
      } else if (i == 2) { // left diagonal
        if (dir == TOP) {
          if (std::abs(wx - cellLeft) < SIDE_TOL &&
              wy >= cellBottom - SIDE_BACK_MARGIN &&
              wy <= cellTop + SIDE_FORWARD_MARGIN)
            wall = LEFT;
        } else if (dir == RIGHT) {
          if (std::abs(wy - cellTop) < SIDE_TOL &&
              wx >= cellLeft - SIDE_BACK_MARGIN &&
              wx <= cellRight + SIDE_FORWARD_MARGIN)
            wall = TOP;
        } else if (dir == LEFT) {
          if (std::abs(wy - cellBottom) < SIDE_TOL &&
              wx >= cellLeft - SIDE_FORWARD_MARGIN &&
              wx <= cellRight + SIDE_BACK_MARGIN)
            wall = DOWN;
        } else if (dir == DOWN) {
          if (std::abs(wx - cellRight) < SIDE_TOL &&
              wy >= cellBottom - SIDE_FORWARD_MARGIN &&
              wy <= cellTop + SIDE_BACK_MARGIN)
            wall = RIGHT;
        }
      } else { // front sensors
        if (dir == TOP) {
          if (std::abs(wy - cellTop) < FRONT_TOL &&
              wx >= cellLeft - FRONT_SIDE_MARGIN &&
              wx <= cellRight + FRONT_SIDE_MARGIN)
            wall = TOP;
        } else if (dir == RIGHT) {
          if (std::abs(wx - cellRight) < FRONT_TOL &&
              wy >= cellBottom - FRONT_SIDE_MARGIN &&
              wy <= cellTop + FRONT_SIDE_MARGIN)
            wall = RIGHT;
        } else if (dir == LEFT) {
          if (std::abs(wx - cellLeft) < FRONT_TOL &&
              wy >= cellBottom - FRONT_SIDE_MARGIN &&
              wy <= cellTop + FRONT_SIDE_MARGIN)
            wall = LEFT;
        } else if (dir == DOWN) {
          if (std::abs(wy - cellBottom) < FRONT_TOL &&
              wx >= cellLeft - FRONT_SIDE_MARGIN &&
              wx <= cellRight + FRONT_SIDE_MARGIN)
            wall = DOWN;
        }
      }

      if (wall == 0)
        continue;

      mouseState.walls[gy][gx] |= wall;

      if ((wall & RIGHT) && gx + 1 < N)
        mouseState.walls[gy][gx + 1] |= LEFT;
      if ((wall & LEFT) && gx - 1 >= 0)
        mouseState.walls[gy][gx - 1] |= RIGHT;
      if ((wall & TOP) && gy + 1 < N)
        mouseState.walls[gy + 1][gx] |= DOWN;
      if ((wall & DOWN) && gy - 1 >= 0)
        mouseState.walls[gy - 1][gx] |= TOP;
    }
  }

  void update(MouseState &mouseState) override {
    updateDt();
    updateSensorState();
    updateEncoders();
    updateWorldCoord();
    updateMazeState(mouseState);
    Serial.printf("COORD: %d, %d  WORLD: %0.2f, %0.2f    WALLS: %d   REL: "
                  "%0.2f, %0.2f \n",
                  getGridCoord().x, getGridCoord().y, w.x, w.y,
                  mouseState.walls[getGridCoord().y][getGridCoord().x],
                  w.gridRelativeCoords(getGridCoord()).x,
                  w.gridRelativeCoords(getGridCoord()).y);
    // Serial.println(analogRead(B_FRONT));
  }

  static void onButtonPress() { Serial.print("PRESSED"); }

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

    pinMode(B_FRONT, INPUT);
    pinMode(B_BACK, INPUT);

    // attachInterrupt(digitalPinToInterrupt(B_FRONT), onButtonPress, RISING);

    gyro.initalizeGyro();

    mLeft.begin();
    mRight.begin();

    Serial.begin(9600);
  };
};
