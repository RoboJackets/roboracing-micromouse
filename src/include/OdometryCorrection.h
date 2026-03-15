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

    WorldCoord deriveCoordFromIR() {
        double sumX = 0;
        double sumY = 0;


        for (int i = 0; i < readings.size(); i++) {

            sumX += readings.at(i).x;
            sumY += readings.at(i).y;
   

        }
        sumX /= readings.size();
        sumY /= readings.size();
        return {sumX, sumY};
        // logic to derive world coord from IR readings
        return {};
    }
};

