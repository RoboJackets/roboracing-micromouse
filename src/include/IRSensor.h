#pragma once
#include <cmath>
#include <cstdint>

#include "Types.h"

constexpr int IR_AVG_WINDOW = 3;

struct IRSensor {
  // (x,y) is position from center, theta is angle of sensor.
  WorldCoord pos_from_center{};
  uint8_t EMIT;
  uint8_t RECV;

  double a;
  double b;

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
    int found = 0;
    for (int i = 0; i < buf_count; i++) {
      if (buffer[i].x == std::numeric_limits<double>::infinity()) {
        continue;
      }
      found++;
      sum_x += buffer[i].x;
      sum_y += buffer[i].y;
    }
    if (found == 0) {
      return {std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity(), pos_from_center.theta};
    }
    return {sum_x / found, sum_y / found, pos_from_center.theta};
  }

  WorldCoord getReading(int post) {
    WorldCoord coord;
    if (post > 20) {
      double dist = a / pow(max(post, 1), b);
      coord = {std::cos(pos_from_center.theta) * dist + pos_from_center.x,
               std::sin(pos_from_center.theta) * dist + pos_from_center.y,
               pos_from_center.theta};
    } else {
      coord = {std::numeric_limits<double>::infinity(),
               std::numeric_limits<double>::infinity(), pos_from_center.theta};
    }
    addReading(coord);
    return coord;
  }
};