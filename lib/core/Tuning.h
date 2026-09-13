#pragma once

#include <cmath>

constexpr double SPEED_FILTER_ALPHA = 0.4;
constexpr double ROTATION_FILTER_ALPHA = 0.4;

constexpr int IR_AVG_WINDOW = 6;

constexpr double OBSERVE_MAX_HEADING_ERROR = 0.2;
constexpr double OBSERVE_MAX_ROTATION_RATE = 0.15;
constexpr double OBSERVE_ENTRY_MARGIN = 0.1;

constexpr double WALL_THRESHOLD_FRONT = 0.075;
constexpr double WALL_THRESHOLD_LEFT = 0.12;
constexpr double WALL_THRESHOLD_RIGHT = 0.11;

constexpr double DRIVE_POS_TOL = 0.01;    // m
constexpr double ROTATION_POS_TOL = 0.02; // rad
constexpr double CURVE_POS_TOL = 0.008;   // m (arc length)
constexpr double VEL_TOL = 0.06;          // m/s

constexpr double DRIVE_ACCEL_SCALE = 0.5;
constexpr double ROTATION_MAX_SPEED_SCALE = 0.1;
constexpr double ROTATION_ACCEL_SCALE = 0.5;
constexpr double CURVE_ACCEL_SCALE = 0.5;
constexpr double CURVE_FRICTION_SPEED_SCALE = 0.3;

constexpr double DRIVE_DEFAULT_MAX_SPEED = 0.1;

constexpr double IR_VALID_RANGE = 0.16;
constexpr double IR_CENTER_OFFSET = 0.005;
constexpr double IR_LEFT_ONLY_SETPOINT = -0.08;
constexpr double IR_RIGHT_ONLY_SETPOINT = 0.095;
constexpr double FRONT_WALL_STOP_DISTANCE = 0.08;

constexpr double YAW_ANGLE_TOL = 4 * M_PI / 180;

constexpr double CELL_STOP_SHORT = 0.01;

struct SpeedProfile {
  double maxSpeed;
  double driveFinalVelocity;
  double curveRadius;
  double curveFinalVelocity;
  double curveTrailDistance;
};

inline constexpr SpeedProfile EXPLORE_SPEED{0.1, 0.1, 0.03, 0.1, 0.03};
inline constexpr SpeedProfile FAST_SPEED{0.2, 0.2, 0.03, 0.2, 0.04};

constexpr double MOTOR_KS = 0.45;
constexpr double MOTOR_KV = 0.7;
constexpr double MOTOR_KA = 0;
