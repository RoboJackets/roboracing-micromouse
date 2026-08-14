#pragma once
#include <array>

#include "DistanceSensors.h"
#include "Drivetrain.h"
#include "MouseIO.h"
#include "Odometry.h"
#include "Types.h"

struct Robot {
  MouseIO &io;
  Odometry odom{};
  Drivetrain drive{};
  DistanceSensors sensors{};

  double lastNow = 0;
  double cachedDt = 0;

  explicit Robot(MouseIO &io) : io(io) {}

  void init();
  void update();

  double getDt() const { return cachedDt; }

  WorldCoord getWorldCoord() const { return odom.pose(); }
  void setWorldCoord(WorldCoord c) { odom.setPose(c); }
  void setGyroOffset(double offset) { odom.setGyroOffset(offset); }
  double getGyroYaw() const { return odom.rawGyroYaw(); }
  double getRotationRate() const { return odom.rotationRate(); }
  double getDriveSpeedLeft() const { return odom.leftSpeed(); }
  double getDriveSpeedRight() const { return odom.rightSpeed(); }

  std::array<WorldCoord, 4> getSensorState() const { return sensors.state(); }
  std::array<WorldCoord, 4> getAverageSensorState() const {
    return sensors.averageState();
  }

  bool buttonPressed() { return io.buttonPressed(); }

  void driveVoltage(double left, double right) {
    drive.setVoltage(io, left, right);
  }
  void driveVelocity(double left, double right) {
    drive.setVelocity(io, left, right, odom.leftSpeed(), odom.rightSpeed(),
                      cachedDt);
  }
  void resetPIDs() { drive.resetPIDs(); }
};
