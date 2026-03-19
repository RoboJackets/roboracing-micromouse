#include <cmath>
#include <cstdint>

#include "Constants.h"
#include "Types.h"

struct EncoderSensor {
  double encoder_a_pin;
  double encoder_b_pin;
  volatile long encoder_counts;
  bool flipped = false;

  void updateEncoder() {
    boolean A = digitalRead(encoder_a_pin) == digitalRead(encoder_b_pin);
    if (A && !flipped || !A && flipped) {
      encoder_counts++; // Clockwise rotation
    } else {
      encoder_counts--; // Counter-clockwise rotation
    }
  }
  double getPosition() {
    return ((encoder_counts / COUNTS_PER_REVOLUTION) * 2 * M_PI *
            WHEEL_RADIUS_M) /
           DRIVE_GEAR_RATIO;
  }
};