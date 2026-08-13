#pragma once
#include <array>

#include "Types.h"

struct MouseIO {
  virtual ~MouseIO() = default;

  virtual WorldCoord getWorldCoord() = 0;
  virtual void updateWorldCoord() = 0;
  virtual void setWorldCoord(WorldCoord c) = 0;

  virtual void driveVoltage(double left, double right) = 0;
  virtual void driveVelocity(double left, double right) = 0;
  virtual double getDriveSpeedLeft() = 0;
  virtual double getDriveSpeedRight() = 0;
  virtual double getDrivePosLeft() = 0;
  virtual double getDrivePosRight() = 0;
  virtual void resetPIDs() = 0;

  virtual double getGyroYaw() = 0;
  virtual double getRotationRate() = 0;
  virtual void setGyroOffset(double offset) = 0;

  virtual std::array<WorldCoord, 4> getSensorState() = 0;
  virtual std::array<WorldCoord, 4> getAverageSensorState() = 0;

  virtual bool buttonPressed() = 0;

  virtual void update() = 0;
  virtual void init() = 0;
  virtual double getDt() = 0;
};
