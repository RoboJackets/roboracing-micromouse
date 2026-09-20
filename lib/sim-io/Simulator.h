#pragma once
#include "MouseIO.h"
#include "RobotState.h"
#include <array>

struct SimIO : MouseIO {
  // each grid represents 1.2cm
  bool worldState[240][240]{};
  double start_time;
  double current_time;
  RobotState robotState;
  
  double sampledYaw = 0;
  double sampledLeft = 0;
  double sampledRight = 0;
  std::array<double, 4> sampledIr{};

  void init() override;

  void poll() override;

  void setMotorPwm(double left, double right) override;

  double leftMeters() const override { return sampledLeft; }
  double rightMeters() const override { return sampledRight; }
  double gyroYaw() const override { return sampledYaw; }
  const std::array<double, 4> &irMeters() const override { return sampledIr; }

  bool buttonPressed() override;
  double now() override;
};
