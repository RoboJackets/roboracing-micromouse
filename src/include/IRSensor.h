#include <cmath>
#include <cstdint>

#include "Types.h"
struct IRSensor {
  // (x,y) is position from center, theta is angle of sensor.
  WorldCoord pos_from_center{};
  uint8_t EMIT;
  uint8_t RECV;

  WorldCoord getReading(double dist) {
    return {std::cos(pos_from_center.theta) * dist - pos_from_center.x,
            std::sin(pos_from_center.theta) * dist - pos_from_center.y};
  }
};