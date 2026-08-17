#pragma once

constexpr double SPEED_FILTER_ALPHA = 0.4;
constexpr double ROTATION_FILTER_ALPHA = 0.4;

constexpr int IR_AVG_WINDOW = 6;

constexpr double OBSERVE_MAX_HEADING_ERROR = 0.2;
constexpr double OBSERVE_MAX_ROTATION_RATE = 0.15;
constexpr double OBSERVE_ENTRY_MARGIN = 0.1;

constexpr double WALL_THRESHOLD_FRONT = 0.075;
constexpr double WALL_THRESHOLD_LEFT = 0.12;
constexpr double WALL_THRESHOLD_RIGHT = 0.11;

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
