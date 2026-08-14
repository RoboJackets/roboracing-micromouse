#pragma once
#include <array>

struct MouseIO {
  virtual ~MouseIO() = default;

  virtual void init() = 0;

  virtual void setMotorPwm(double left, double right) = 0;

  virtual double leftMeters() = 0;
  virtual double rightMeters() = 0;
  virtual double gyroYaw() = 0;
  virtual std::array<double, 4> irMeters() = 0;

  virtual bool buttonPressed() = 0;
  virtual double now() = 0;
};
