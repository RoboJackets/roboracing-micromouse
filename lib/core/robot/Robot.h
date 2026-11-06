#pragma once
#include <array>

#include "DistanceSensors.h"
#include "Drivetrain.h"
#include "MouseIO.h"
#include "Odometry.h"
#include "Types.h"
#include "maze/MazeMap.h"
#include "ControlAlgorithms.h"

struct WheelSpeeds {
  double left;
  double right;
};

class MotionProfile {
  TrapezoidalProfile profile;
  TrapezoidalProfile::State target;
  TrapezoidalProfile::State ref;
  static constexpr double POS_EPS = 1e-9;
  bool done = true;
  double dir = 1.0;

public:
  MotionProfile() : profile{{0.0, 0.0}} {

  }

  void startMove(double position, double finalVelocity, TrapezoidalProfile::Constraints constraints) {
    profile.cts = constraints;
    target.position += position;
    target.velocity = finalVelocity;
    dir = position >= 0 ? 1.0 : -1.0;
    done = false;
  }

  void hold() {
    target = {ref.position, 0};
    ref.velocity = 0;
    done = true;
  }

  void update(double dt) {
    if (done) {
      ref.position += ref.velocity * dt;
      return;
    }
    ref = profile.calculate(dt, ref, target);
    bool reached = (target.position - ref.position) * dir <= POS_EPS;
    if (reached) {
      ref = target;
      done = true;
    }
  }

  double position() {
    return ref.position;
  }

  double velocity() {
    return ref.velocity;
  }

  // Put the reference at rest on the measured value.
  void reset(double measured) {
    ref = target = {measured, 0};
    done = true;
  }

  bool finished() const { return done; }
};

struct MotionController {
  MotionProfile forward;
  PID forwardPID;

  MotionProfile rotation;
  PID rotationPID;

  double forwardError = 0;
  double rotationError = 0;

  MotionController(PIDConstants forward, PIDConstants rotation) :
    forward{},
    forwardPID{forward},
    rotation{},
    rotationPID{rotation} {

  }

  WheelSpeeds update(double dt, double distance, double angle) {
    forward.update(dt);
    rotation.update(dt);
    forwardError = forward.position() - distance;
    rotationError = rotation.position() - angle;
    double v = forward.velocity() + forwardPID.calculate(distance, forward.position(), dt);
    double w = rotation.velocity() + rotationPID.calculate(angle, rotation.position(), dt);
    return {v - w * WHEEL_SEPARATION_M / 2, v + w * WHEEL_SEPARATION_M / 2};
  }

  bool enabled = false;

  void reset(double distance, double angle) {
    forward.reset(distance);
    rotation.reset(angle);
    forwardPID.reset();
    rotationPID.reset();
    forwardError = 0;
    rotationError = 0;
  }

  bool settled(double measuredSpeed) const {
    return std::abs(forwardError) < DRIVE_POS_TOL &&
           std::abs(rotationError) < ROTATION_POS_TOL &&
           measuredSpeed < VEL_TOL;
  }
};

struct Robot {
  MouseIO &io;
  Odometry odom{};
  MotionController controller{profilePIDConstants, profilePIDConstants};
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
