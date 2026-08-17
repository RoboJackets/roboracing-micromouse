#include "Drivetrain.h"

#include <algorithm>

void Drivetrain::setDuty(MouseIO &io, double left, double right) {
  io.setMotorPwm(std::clamp(left, -1.0, 1.0), std::clamp(right, -1.0, 1.0));
}

void Drivetrain::setVelocity(MouseIO &io, double left, double right,
                             double leftSpeed, double rightSpeed, double dt) {
  setDuty(io,
          leftff.calculate(left, dt) +
              velocityPIDLeft.calculate(leftSpeed, left, dt),
          rightff.calculate(right, dt) +
              velocityPIDRight.calculate(rightSpeed, right, dt));
}

void Drivetrain::resetPIDs() {
  velocityPIDLeft.reset();
  velocityPIDRight.reset();
  leftff.reset();
  rightff.reset();
}
