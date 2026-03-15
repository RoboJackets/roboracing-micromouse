#include "Helpers.h"
#include "Types.h"
#include <cmath>
#include <cstdint>
#include "IRSensor.h"
#include "Pins.h"
#include "Constants.h"
#include <Arduino.h>
#include <DRV8833.h>

struct OdometeryCorrection {
    WorldCoord cur{};
    std::vector<WorldCoord> readings{};

    double getError() {
        WorldCoord derived = deriveCoordFromIR();
        double errorX = derived.x - cur.x;
        double errorY = derived.y - cur.y;
        return std::hypot(errorX, errorY);
    }

    WorldCoord deriveCoordFromIR()
    {
        if (readings.empty())
            return cur;

        double wx = 0, wy = 0, totalWeight = 0;
        auto ref = *std::min_element(readings.begin(), readings.end(),
                                     [](WorldCoord &a, WorldCoord &b)
                                     {
                                         return a.hypot() < b.hypot();
                                     });

        for (size_t i = 0; i < readings.size(); i++)
        {
            if (sameObject(readings[i], ref, 0.1))
            {
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

    bool sameObject(WorldCoord readingA, WorldCoord readingB, double threshold)
    {
        double dist = std::hypot(readingA.x - readingB.x, readingA.y - readingB.y);
        return dist < threshold;
    }
};

