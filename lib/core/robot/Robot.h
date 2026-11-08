#pragma once
#include <array>

#include "DistanceSensors.h"
#include "Drivetrain.h"
#include "MouseIO.h"
#include "Odometry.h"
#include "Types.h"
#include "maze/MazeMap.h"
#include "Constants.h"
#include "Tuning.h"
#include "control/ControlAlgorithms.h"
#include "control/Motion.h"

struct Robot {
  MouseIO &io;
  Odometry odom{};
  MotionController controller{profilePIDConstants, profilePIDConstants,
                              DRIVE_POS_TOL,       ROTATION_POS_TOL,
                              VEL_TOL,             WHEEL_SEPARATION_M};
  Drivetrain drive{};
  DistanceSensors sensors{};
  MazeMap map{};

  double lastNow = 0;
  double cachedDt = 0;

  explicit Robot(MouseIO &io) : io(io) {}

  void init();
  void update();

  double getDt() const { return cachedDt; }

  Translation2d getWorldCoord() const { return odom.pose(); }
  double getHeading() const { return odom.heading(); }
  double getUnwrappedHeading() const { return odom.unwrappedHeading(); }
  double getDistance() const {
    return (odom.leftPosition + odom.rightPosition) / 2.0;
  }
  void startController() {
    controller.reset(getDistance(), getUnwrappedHeading());
    controller.enabled = true;
  }
  Pose2d getPose() const { return Pose2d{odom.pose(), odom.heading()}; }
  void setWorldCoord(Translation2d c) { odom.setPose(c); }
  void setGyroOffset(double offset) { odom.setGyroOffset(offset); }
  double getGyroYaw() const { return odom.rawGyroYaw(); }
  double getRotationRate() const { return odom.rotationRate(); }
  double getDriveSpeedLeft() const { return odom.leftSpeed(); }
  double getDriveSpeedRight() const { return odom.rightSpeed(); }
  Translation2d getDriveSpeed() const {
    return {odom.leftSpeed(), odom.rightSpeed()};
  }

  const std::array<Translation2d, 4> &getSensorState() const {
    return sensors.state();
  }
  const std::array<Translation2d, 4> &getAverageSensorState() const {
    return sensors.averageState();
  }

  bool buttonPressed() { return io.buttonPressed(); }

  void observe(Cell at) {
    map.observe(at, getPose(), odom.rotationRate(), sensors.state());
  }

  void driveDuty(double left, double right) { drive.setDuty(io, left, right); }
  void driveVelocity(double left, double right) {
    drive.setVelocity(io, left, right, odom.leftSpeed(), odom.rightSpeed(),
                      cachedDt);
  }
  void resetPIDs() { drive.resetPIDs(); }
};
