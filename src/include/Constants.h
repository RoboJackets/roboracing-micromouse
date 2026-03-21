#pragma once

#include <cmath>
#include <cstdint>

constexpr uint32_t EMIT_RECV_DELAY_US = 300;

constexpr double WHEEL_RADIUS_M = (0.032 / 2);

constexpr double COUNTS_PER_REVOLUTION = 12;

constexpr double COUNTS_PER_WHEEL_REV = 616.4;

constexpr double ROBOT_LENGTH = 0.13;

constexpr double CELL_SIZE_METERS = 0.18;
constexpr double MAX_SPEED_M_S = 1.5;
constexpr double MAX_ACCEL_M_S2 = 1;
constexpr double WHEEL_SEPERATION_M = 0.1;
constexpr double MAX_ROT_SPEED_RAD_S = (2 * MAX_SPEED_M_S) / WHEEL_SEPERATION_M;
constexpr double MAX_ROT_SPEED_RAD_S2 =
    (2 * MAX_ACCEL_M_S2) / WHEEL_SEPERATION_M;
constexpr double COEF_FRICTION = 0.5;
constexpr double DRIVE_GEAR_RATIO = 29.86;
constexpr double MASS = 1;
constexpr double MAX_CURVE_CF = MASS * 9.81 * COEF_FRICTION;
const double CURVE_VELOCITY =
    std::sqrt((MAX_CURVE_CF * CELL_SIZE_METERS / 2) / MASS);

constexpr double FRONT_SENSOR_SEP = 1;
constexpr double GYRO_ALPHA = 0.98;

constexpr double TURN_RADIUS = 0.04;