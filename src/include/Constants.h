#pragma once

#include <cstdint>

constexpr uint32_t EMIT_RECV_DELAY_US = 10'000;

double WHEEL_RADIUS_M = 0.1;

double COUNTS_PER_REVOLUTION = 1;

double CELL_SIZE_METERS = 0.2;
double MAX_SPEED_M_S = 10;
double MAX_ACCEL_M_S2 = 5;
double WHEEL_SEPERATION_M = 0.1;
double MAX_ROT_SPEED_RAD_S = (2 * MAX_SPEED_M_S) / WHEEL_SEPERATION_M;
double MAX_ROT_SPEED_RAD_S2 = (2 * MAX_ACCEL_M_S2) / WHEEL_SEPERATION_M;
double COEF_FRICTION = 1;
double DRIVE_G_RATIO = 1;