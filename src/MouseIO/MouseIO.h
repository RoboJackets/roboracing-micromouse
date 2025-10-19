#pragma once
#include "IdealState.h"
#include "Mouse.h"
#include "Types.h"
struct MouseIO {
  virtual GridCoord getGridCoord() = 0;
  virtual unsigned char getGridDir() = 0;
  virtual WorldCoord getWorldCoord() = 0;
  virtual void updateWorldCoord() = 0;

  virtual void drive(double left, double right) = 0;
  virtual double getDriveSpeedLeft() = 0;
  virtual double getDriveSpeedRight() = 0;
  virtual double getDrivePosLeft() = 0;
  virtual double getDrivePosRight() = 0;
  virtual double getGyroYaw() = 0;
  virtual void setState(IdealState state) = 0;
  virtual void getSensorState() = 0;

  virtual void update(MouseState& mouseState) = 0;

  virtual void init() = 0;
};