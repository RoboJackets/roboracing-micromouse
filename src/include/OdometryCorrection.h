#include "Constants.h"
#include "Helpers.h"
#include "IRSensor.h"
#include "Pins.h"
#include "Types.h"
#include <Arduino.h>
#include <DRV8833.h>
#include <cmath>
#include <cstdint>


struct OdometeryCorrection {
  WorldCoord cur{};
  std::vector<WorldCoord> readings{};

  double getError() {
    WorldCoord derived = deriveCoordFromIR();
    double errorX = derived.x - cur.x;
    double errorY = derived.y - cur.y;
    return std::hypot(errorX, errorY);
  }

  WorldCoord deriveCoordFromIR() {
    if (readings.empty())
      return cur;

    double wx = 0, wy = 0, totalWeight = 0;
    // Find the closest reading as a reference point
    auto ref = *std::min_element(
        readings.begin(), readings.end(),
        [](WorldCoord &a, WorldCoord &b) { return a.hypot() < b.hypot(); });

    for (size_t i = 0; i < readings.size(); i++) {
      if (sameObject(readings[i], ref, 0.1)) {
        double w = 1.0 / (readings[i].hypot() + 0.01);
        wx += readings[i].x * w;
        wy += readings[i].y * w;
        totalWeight += w;
      }
    }

    if (totalWeight == 0)
      return cur;
    wx /= totalWeight;
    wy /= totalWeight;
    return {wx, wy, cur.theta};
  }

  bool sameObject(WorldCoord readingA, WorldCoord readingB, double threshold) {
    double dist = std::hypot(readingA.x - readingB.x, readingA.y - readingB.y);
    return dist < threshold;
  }

  WorldCoord correct() {
    double error = getError();
    double trust = IR_SENSOR_TRUST;
    double correctedX =
        (1 - trust) * cur.x + trust * (deriveCoordFromIR().x - cur.x);
    double correctedY =
        (1 - trust) * cur.y + trust * (deriveCoordFromIR().y - cur.y);
    return {correctedX, correctedY, cur.theta};
  }
};
