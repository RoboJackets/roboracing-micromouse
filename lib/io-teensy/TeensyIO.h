#pragma once
#include <array>
#include <vector>

#include "Constants.h"
#include "DRV8833.h"
#include "EncoderSensor.h"
#include "Gyro.h"
#include "IRSensor.h"
#include "MouseIO.h"
#include "Pins.h"

struct TeensyIO : MouseIO {
  // FL, FR, DL, DR
  std::vector<IRSensor> sensors{IRSensor{EMIT_1, RECV_1, 0.968202, 0.500721},
                                IRSensor{EMIT_4, RECV_4, 1.30488, 0.547281},
                                IRSensor{EMIT_2, RECV_2, 1.30488, 0.547281},
                                IRSensor{EMIT_3, RECV_3, 0.818039, 0.53409}};
  EncoderSensor encoderLeft{ACODER_a, ACODER_b, 0, true};
  EncoderSensor encoderRight{BCODER_a, BCODER_b, 0, false};

  Gyro gyro{};

  uint32_t lastMicros = 0;
  uint64_t microsAccum = 0;

  DRV8833Motor mLeft = DRV8833Motor(AIN1, AIN2, 1, STBY);
  DRV8833Motor mRight = DRV8833Motor(BIN1, BIN2, 1, STBY);

  void init() override;

  void setMotorPwm(double left, double right) override;

  double leftMeters() override;
  double rightMeters() override;
  double gyroYaw() override;
  std::array<double, 4> irMeters() override;

  bool buttonPressed() override;
  double now() override;
};
