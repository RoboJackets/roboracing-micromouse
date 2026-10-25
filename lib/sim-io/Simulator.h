#pragma once
#include "MouseIO.h"
#include "RobotState.h"
#include "maze/Maze.h"
#include <array>
#include <chrono>
#include <ostream>
#include <string>

struct SimIO : MouseIO {
  // each grid represents 0.012m
  bool worldState[240][240]{};
  double dt = 0.001;
  double current_time = 0;
  double current_left_pwm{0};
  double current_right_pwm{0};
  RobotState robotState{};
  
  double sampledYaw = 0;
  double sampledLeft = 0;
  double sampledRight = 0;
  std::array<double, 4> sampledIr{};

  // Marks the wall on side d of cell (cx, cy), one grid square thick.
  void addWall(int cx, int cy, Dir d);
  // Replaces worldState with the maze in a .num file ("x y N E S W" per cell,
  // 1 = wall). Returns false if the file can't be opened.
  bool loadMazeNum(std::ifstream& in);
  // Prints worldState as one line: "MAZE: 240 240 " followed by 240 * 240
  // '0'/'1' characters, row by row from y = 0, x = 0..239 within each row.
  void printMaze(std::ostream &out) const;

  void init() override;

  void poll() override;

  void setMotorPwm(double left, double right) override;

  double leftMeters() const override { return sampledLeft; }
  double rightMeters() const override { return sampledRight; }
  double gyroYaw() const override { return sampledYaw; }
  const std::array<double, 4> &irMeters() const override { return sampledIr; }

  bool buttonPressed() override;
  double now() override;
  int printf(const char *, ...) override;
};
