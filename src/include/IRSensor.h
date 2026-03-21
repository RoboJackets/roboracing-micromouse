#pragma once
#include <cmath>
#include <cstdint>

#include "Types.h"

constexpr int IR_AVG_WINDOW = 5;

struct IRSensor {
  // (x,y) is position from center, theta is angle of sensor.
  WorldCoord pos_from_center{};
  uint8_t EMIT;
  uint8_t RECV;

  double a = 0.647426;
  double b = 0.516999;

  WorldCoord buffer[IR_AVG_WINDOW]{};
  int buf_index = 0;
  int buf_count = 0;

  void addReading(WorldCoord coord) {
    buffer[buf_index] = coord;
    buf_index = (buf_index + 1) % IR_AVG_WINDOW;
    if (buf_count < IR_AVG_WINDOW)
      buf_count++;
  }

  WorldCoord getAverage() {
    double sum_x = 0, sum_y = 0;
    for (int i = 0; i < buf_count; i++) {
      sum_x += buffer[i].x;
      sum_y += buffer[i].y;
    }
    return {sum_x / buf_count, sum_y / buf_count, pos_from_center.theta};
  }

  WorldCoord getReading(int post) {
    double dist = post < 4 ? std::numeric_limits<double>::infinity()
                           : 0.647426 / pow(max(post, 1), 0.516999);
    WorldCoord coord = {
        std::cos(pos_from_center.theta) * dist - pos_from_center.x,
        std::sin(pos_from_center.theta) * dist - pos_from_center.y,
        pos_from_center.theta};
    addReading(coord);
    return getAverage();
  }
};