#pragma once
#include <vector>

#include "IdealState.h"
#include "Mouse.h"
#include "Types.h"
struct MouseIO {
  virtual bool isMMS() const { return false; }
  virtual GridCoord getGridCoord() {};
  virtual unsigned char getGridDir() {};
  virtual WorldCoord getWorldCoord() {};
  virtual void updateWorldCoord() {};

  virtual void drive(double left, double right) {};
  virtual double getDriveSpeedLeft() { return 0; };
  virtual double getDriveSpeedRight() { return 0; };
  virtual double getDrivePosLeft() { return 0; };
  virtual double getDrivePosRight() { return 0; };
  virtual double getGyroYaw() { return 0; };
  virtual void setState(IdealState state) {};
  virtual std::vector<WorldCoord> getSensorState() { return {}; };

  virtual void update(MouseState& mouseState) {};

  virtual void init() {};
};