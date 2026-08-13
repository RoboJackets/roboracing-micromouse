#include "TeensyIO.h"
#include <Arduino.h>

void TeensyIO::resetPIDs() {
  velocityPIDLeft.resetAccum();
  velocityPIDRight.resetAccum();
}

void TeensyIO::updateWorldCoord() {
  double deltaLeft = getDrivePosLeft() - lastLeftPosition;
  double deltaRight = getDrivePosRight() - lastRightPosition;
  double wheelDelta = ((deltaLeft + deltaRight) / 2);

  // if (readings.size() >= 2 && readings.at(0).hypot() < 0.18 &&
  //     readings.at(1).hypot() < 0.18) {
  //   double deltaR = readingsAverage.at(0).y - readingsAverage.at(1).y;
  //   double sensorYaw = std::atan2(deltaR, FRONT_SENSOR_SEP);
  //   double currentHeading = gyroYaw - gyroOffset;
  //   double nearestCardinal =
  //       std::round(currentHeading / (M_PI / 2.0)) * (M_PI / 2.0);
  //   double sensorOffset = gyroYaw - nearestCardinal - sensorYaw;
  //   gyroOffset = GYRO_ALPHA * gyroOffset + (1.0 - GYRO_ALPHA) *
  //   sensorOffset;
  // }
  double theta = (-getGyroYaw() - gyroOffset);
  double deltaX = wheelDelta * std::cos(theta);
  double deltaY = wheelDelta * std::sin(theta);

  w = WorldCoord{w.x + deltaX, w.y + deltaY, theta};
}

void TeensyIO::updateEncoders() {
  lastLeftPosition = leftPosition;
  lastRightPosition = rightPosition;
  leftPosition = encoderLeft.getPosition();
  rightPosition = encoderRight.getPosition();

  double dt = getDt();
  double rawLeft = (leftPosition - lastLeftPosition) / dt;
  double rawRight = (rightPosition - lastRightPosition) / dt;

  constexpr double alpha = 0.4;
  filteredSpeedLeft += alpha * (rawLeft - filteredSpeedLeft);
  filteredSpeedRight += alpha * (rawRight - filteredSpeedRight);
}

void TeensyIO::driveVoltage(double left, double right) {
  double l = std::clamp(left, -1.0, 1.0);
  double r = std::clamp(right, -1.0, 1.0);
  if (l == 0.0 && r == 0.0) {
    mLeft.brake();
    mRight.brake();
  } else {
    mLeft.drive((int)(l * 255));
    mRight.drive((int)(r * 255));
  }
}

void TeensyIO::driveVelocity(double left, double right) {
  driveVoltage(
      leftff.calculate(left, getDt()) +
          velocityPIDLeft.calculate(getDriveSpeedLeft(), left, getDt()),
      rightff.calculate(right, getDt()) +
          velocityPIDRight.calculate(getDriveSpeedRight(), right, getDt()));
}

void TeensyIO::updateDt() {
  uint32_t now = micros();
  uint32_t deltaMicros = now - lastMicros;
  lastMicros = now;
  cachedDt = std::max(deltaMicros * 1e-6, 1e-6);
}

void TeensyIO::updateSensorState() {
  for (size_t i = 0; i < sensors.size(); i++) {
    IRSensor &sensor = sensors.at(i);
    digitalWrite(sensor.EMIT, HIGH);
    delayMicroseconds(EMIT_RECV_DELAY_US);
    int post = analogRead(sensor.RECV);
    digitalWrite(sensor.EMIT, LOW);
    
    readings[i] = sensor.getReading(post);
    readingsAverage[i] = sensor.getAverage();
    Serial.print(i);
    Serial.print(": ");
    Serial.printf("%0.2f, %0.2f", readings[i].x, readings[i].y);
    Serial.print(post);
    Serial.print("     ");
  }
  gyro.update();
  double prevYaw = gyroYaw;
  gyroYaw = gyro.ypr[0];

  double rawRotationRate = (gyroYaw - prevYaw) / getDt();
  constexpr double rotAlpha = 0.4;
  filteredRotationRate += rotAlpha * (rawRotationRate - filteredRotationRate);
}

void TeensyIO::update() {
  updateDt();
  updateSensorState();
  updateEncoders();
  updateWorldCoord();
  Serial.printf("WORLD: %0.2f, %0.2f  THETA: %0.2f\n", w.x, w.y, w.theta);
}

bool TeensyIO::buttonPressed() {
  return !digitalRead(B_FRONT) || !digitalRead(B_BACK);
}

void TeensyIO::init() {
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

  pinMode(B_FRONT, INPUT);
  pinMode(B_BACK, INPUT);

  gyro.initalizeGyro();

  mLeft.begin();
  mRight.begin();

  Serial.begin(9600);
}
