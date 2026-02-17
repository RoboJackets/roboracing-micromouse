#include <cmath>
#include <cstdint>
#include <Encoder.h>

#include "Types.h"
#include "Constants.h"

struct EncoderSensor {
    double encoder_a_pin;
    double encoder_b_pin;
    volatile long encoder_counts;


    void updateEncoder()
    {
        if (digitalRead(encoder_a_pin) == digitalRead(encoder_b_pin))
        {
            encoder_counts++; // Clockwise rotation
        }
        else
        {
            encoder_counts--; // Counter-clockwise rotation
        }
    }
    double getPosition() {
        return (encoder_counts / COUNTS_PER_REVOLUTION) * 2 * M_PI * WHEEL_RADIUS_M;
    }
};