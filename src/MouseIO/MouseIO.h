#pragma once
#include <array>

#include "Mouse.h"
#include "Types.h"

struct MouseIO {
  virtual ~MouseIO() = default;

  // Odometry
  virtual GridCoord getGridCoord() = 0;
  virtual unsigned char getGridDir() = 0;
  virtual WorldCoord getWorldCoord() = 0;
  virtual void updateWorldCoord() = 0;
  virtual void setWorldCoord(WorldCoord c) = 0;

  // Drivetrain
  virtual void driveVoltage(double left, double right) = 0;
  virtual void driveVelocity(double left, double right) = 0;
  virtual double getDriveSpeedLeft() = 0;
  virtual double getDriveSpeedRight() = 0;
  virtual double getDrivePosLeft() = 0;
  virtual double getDrivePosRight() = 0;
  virtual void resetPIDs() = 0;

  // Gyro
  virtual double getGyroYaw() = 0;
  virtual double getRotationRate() = 0;
  virtual void setGyroOffset(double offset) = 0;

  // Distance sensors
  virtual std::array<WorldCoord, 4> getSensorState() = 0;
  virtual std::array<WorldCoord, 4> getAverageSensorState() = 0;

  // Mapping + timing
  virtual void allowUpdates(bool x) = 0;
  virtual void updateMazeState(MouseState &mouseState) = 0;
  virtual void update(MouseState &mouseState) = 0;
  virtual void init() = 0;
  virtual double getDt() = 0;
};
