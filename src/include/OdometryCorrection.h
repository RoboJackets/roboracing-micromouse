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

    }

    double deriveCoordFromIR() {
        for (int i = 0; i < readings.size(); i++) {
            Serial.print("This is x from");
            Serial.print(i);
            Serial.println(readings[i].x);
            Serial.print("This is y from");
            Serial.print(i);
            Serial.println(readings[i].y);
        }
    }
};

