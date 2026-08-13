#pragma once
#include <array>
#include <cmath>
#include <vector>

#include "Constants.h"
#include "ControlAlgorithms.h"
#include "DRV8833.h"
#include "EncoderSensor.h"
#include "Gyro.h"
#include "IRSensor.h"
#include "MouseIO.h"
#include "Pins.h"
#include "Types.h"

struct TeensyIO : MouseIO {
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
      IRSensor{{-0.0473, 0.013, M_PI / 2}, EMIT_1, RECV_1, 0.968202, 0.500721},
      IRSensor{{0.0473, 0.013, M_PI / 2}, EMIT_4, RECV_4, 1.30488, 0.547281},
      IRSensor{{-0.021, 0.038, M_PI}, EMIT_2, RECV_2, 1.30488, 0.547281},
      IRSensor{{0.021, 0.038, 0}, EMIT_3, RECV_3, 0.818039, 0.53409}};
  EncoderSensor encoderLeft{ACODER_a, ACODER_b, 0, true};
  EncoderSensor encoderRight{BCODER_a, BCODER_b, 0, false};
  std::array<WorldCoord, 4> readings{};
  std::array<WorldCoord, 4> readingsAverage{};
  double gyroOffset = 0;

  double filteredSpeedLeft = 0;
  double filteredSpeedRight = 0;
  double filteredRotationRate = 0;
  double lastGyroYaw = 0;

  Gyro gyro{};
  PID velocityPIDRight{velocityPIDConstants};
  PID velocityPIDLeft{velocityPIDConstants};

  MotorFeedForward leftff{0.45, 0.7, 0};
  MotorFeedForward rightff{0.45, 0.7, 0};

  DRV8833Motor mLeft = DRV8833Motor(AIN1, AIN2, 1, STBY);
  DRV8833Motor mRight = DRV8833Motor(BIN1, BIN2, 1, STBY);

  void resetPIDs() override;

  WorldCoord getWorldCoord() override { return w; }
  void updateWorldCoord() override;
  void setWorldCoord(WorldCoord c) override { w = c; }

  void updateEncoders();

  void driveVoltage(double left, double right) override;
  void driveVelocity(double left, double right) override;
  void setGyroOffset(double offset) override { gyroOffset = offset; }

  double getDriveSpeedLeft() override { return filteredSpeedLeft; }
  double getDriveSpeedRight() override { return filteredSpeedRight; }
  double getRotationRate() override { return filteredRotationRate; }

  double getDrivePosLeft() override { return leftPosition; }
  double getDrivePosRight() override { return rightPosition; }
  double getGyroYaw() override { return gyroYaw; }

  std::array<WorldCoord, 4> getSensorState() override { return readings; }
  std::array<WorldCoord, 4> getAverageSensorState() override {
    return readingsAverage;
  }

  bool buttonPressed() override;

  void updateDt();
  double getDt() override { return cachedDt; }

  void updateSensorState();
  void update() override;
  void init() override;
};
