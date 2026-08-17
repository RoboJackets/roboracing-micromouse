#include "DistanceSensors.h"

#include <cmath>
#include <limits>

namespace {
constexpr double INF_M = std::numeric_limits<double>::infinity();

WorldCoord outOfRange(const SensorMount &m) {
  return WorldCoord{std::copysign(INF_M, std::cos(m.theta)),
                    std::copysign(INF_M, std::sin(m.theta)), m.theta};
}
} // namespace

void DistanceSensors::update(const std::array<double, 4> &metres) {
  for (int i = 0; i < 4; ++i) {
    const SensorMount &m = IR_MOUNTS[i];
    WorldCoord coord;
    if (std::isinf(metres[i])) {
      coord = outOfRange(m);
    } else {
      coord = {std::cos(m.theta) * metres[i] + m.x,
               std::sin(m.theta) * metres[i] + m.y, m.theta};
    }

    buffer[i][bufIndex[i]] = coord;
    bufIndex[i] = (bufIndex[i] + 1) % IR_AVG_WINDOW;
    if (bufCount[i] < IR_AVG_WINDOW)
      bufCount[i]++;

    double sumX = 0, sumY = 0;
    int found = 0;
    for (int j = 0; j < bufCount[i]; ++j) {
      if (std::isinf(buffer[i][j].x))
        continue;
      found++;
      sumX += buffer[i][j].x;
      sumY += buffer[i][j].y;
    }

    readings[i] = coord;
    averages[i] = found == 0
                      ? outOfRange(m)
                      : WorldCoord{sumX / found, sumY / found, m.theta};
  }
}
