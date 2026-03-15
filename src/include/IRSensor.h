#include <cmath>
#include <cstdint>

#include "Types.h"

constexpr int IR_AVG_WINDOW = 5;

struct IRSensor {
  // (x,y) is position from center, theta is angle of sensor.
  WorldCoord pos_from_center{};
  uint8_t EMIT;
  uint8_t RECV;

  double buffer[IR_AVG_WINDOW]{};
  int buf_index = 0;
  int buf_count = 0;

  void addReading(double dist) {
    buffer[buf_index] = dist;
    buf_index = (buf_index + 1) % IR_AVG_WINDOW;
    if (buf_count < IR_AVG_WINDOW) buf_count++;
  }

  double getAverage() {
    double sum = 0;
    for (int i = 0; i < buf_count; i++) {
      sum += buffer[i];
    }
    return sum / buf_count;
  }

  WorldCoord getReading(double dist) {
    addReading(dist);
    double avg = getAverage();
    return {std::cos(pos_from_center.theta) * avg - pos_from_center.x,
            std::sin(pos_from_center.theta) * avg - pos_from_center.y};
  }
};