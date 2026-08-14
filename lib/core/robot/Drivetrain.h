#pragma once
#include "ControlAlgorithms.h"
#include "MouseIO.h"
#include "Tuning.h"

struct Drivetrain {
  PID velocityPIDLeft{velocityPIDConstants};
  PID velocityPIDRight{velocityPIDConstants};

  MotorFeedForward leftff{MOTOR_KS, MOTOR_KV, MOTOR_KA};
  MotorFeedForward rightff{MOTOR_KS, MOTOR_KV, MOTOR_KA};

  void setVoltage(MouseIO &io, double left, double right);
  void setVelocity(MouseIO &io, double left, double right, double leftSpeed,
                   double rightSpeed, double dt);
  void resetPIDs();
};
