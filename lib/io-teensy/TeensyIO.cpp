#include "TeensyIO.h"

#include <Arduino.h>

void TeensyIO::setMotorPwm(double left, double right) {
  if (left == 0.0 && right == 0.0) {
    mLeft.brake();
    mRight.brake();
  } else {
    mLeft.drive((int)(left * 255));
    mRight.drive((int)(right * 255));
  }
}

double TeensyIO::leftMeters() { return encoderLeft.getPosition(); }
double TeensyIO::rightMeters() { return encoderRight.getPosition(); }

double TeensyIO::gyroYaw() {
  gyro.update();
  return gyro.ypr[0];
}

std::array<double, 4> TeensyIO::irMeters() {
  std::array<double, 4> out{};
  for (size_t i = 0; i < sensors.size(); i++) {
    IRSensor &sensor = sensors.at(i);
    digitalWrite(sensor.EMIT, HIGH);
    delayMicroseconds(EMIT_RECV_DELAY_US);
    int post = analogRead(sensor.RECV);
    digitalWrite(sensor.EMIT, LOW);
    out[i] = sensor.metersFrom(post);
  }
  return out;
}

bool TeensyIO::buttonPressed() {
  return !digitalRead(B_FRONT) || !digitalRead(B_BACK);
}

double TeensyIO::now() {
  const uint32_t m = micros();
  microsAccum += static_cast<uint32_t>(m - lastMicros);
  lastMicros = m;
  return microsAccum * 1e-6;
}

void TeensyIO::init() {
  Serial.begin(9600);

  lastMicros = micros();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(EMIT_1, OUTPUT);
  pinMode(EMIT_2, OUTPUT);
  pinMode(EMIT_3, OUTPUT);
  pinMode(EMIT_4, OUTPUT);

  pinMode(RECV_1, INPUT);
  pinMode(RECV_2, INPUT);
  pinMode(RECV_3, INPUT);
  pinMode(RECV_4, INPUT);
  analogReadResolution(10);

  pinMode(B_FRONT, INPUT);
  pinMode(B_BACK, INPUT);

  gyro.initializeGyro();

  mLeft.begin();
  mRight.begin();
}
