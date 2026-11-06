#include "Motion.h"

#include <cmath>

MotionProfile::MotionProfile() : profile{{0.0, 0.0}} {}

void MotionProfile::startMove(double position, double finalVelocity,
                              TrapezoidalProfile::Constraints constraints) {
  profile.cts = constraints;
  target.position += position;
  target.velocity = finalVelocity;
  dir = position >= 0 ? 1.0 : -1.0;
  done = false;
}

void MotionProfile::hold() {
  target = {ref.position, 0};
  ref.velocity = 0;
  done = true;
}

void MotionProfile::update(double dt) {
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

void MotionProfile::reset(double measured) {
  ref = target = {measured, 0};
  done = true;
}

MotionController::MotionController(PIDConstants forward, PIDConstants rotation,
                                   double driveTol, double rotTol,
                                   double speedTol, double wheelSeparation)
    : forward{},
      forwardPID{forward},
      rotation{},
      rotationPID{rotation},
      driveTol{driveTol},
      rotTol{rotTol},
      speedTol{speedTol},
      wheelSeparation{wheelSeparation} {}

WheelSpeeds MotionController::update(double dt, double distance, double angle) {
  forward.update(dt);
  rotation.update(dt);
  forwardError = forward.position() - distance;
  rotationError = rotation.position() - angle;
  double v = forward.velocity() + forwardPID.calculate(distance, forward.position(), dt);
  double w = rotation.velocity() + rotationPID.calculate(angle, rotation.position(), dt);
  return {v - w * wheelSeparation / 2, v + w * wheelSeparation / 2};
}

void MotionController::reset(double distance, double angle) {
  forward.reset(distance);
  rotation.reset(angle);
  forwardPID.reset();
  rotationPID.reset();
  forwardError = 0;
  rotationError = 0;
}

bool MotionController::settled(double measuredSpeed) const {
  return std::abs(forwardError) < driveTol &&
         std::abs(rotationError) < rotTol &&
         measuredSpeed < speedTol;
}
