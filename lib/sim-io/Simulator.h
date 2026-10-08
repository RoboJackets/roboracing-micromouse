#pragma once
#include "MouseIO.h"
#include "RobotState.h"
#include <array>
#include <chrono>

struct SimIO : MouseIO {
  // each grid represents 1.2cm
  bool worldState[240][240];
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  double current_time = 0;
  double current_left_pwm{0};
  double current_right_pwm{0};
  RobotState robotState{};
  
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
