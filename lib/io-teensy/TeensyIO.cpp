#include "TeensyIO.h"
#include <Arduino.h>

GridCoord TeensyIO::getGridCoord() {
  int gx = std::floor(w.x / CELL_SIZE_METERS);
  int gy = std::floor(w.y / CELL_SIZE_METERS);
  unsigned char dir = getGridDir(w.theta);
  // TODO: update gyro
  return GridCoord{gx, gy, dir};
}

void TeensyIO::resetPIDs() {
  velocityPIDLeft.resetAccum();
  velocityPIDRight.resetAccum();
}

unsigned char TeensyIO::getGridDir(double angle) {
  double deg = std::fmod(angle * 180.0 / M_PI, 360.0);
  if (deg < 0)
    deg += 360;
  if (deg >= 315 || deg < 45)
    return RIGHT;
  if (deg < 135)
    return TOP;
  if (deg < 225)
    return LEFT;
  return DOWN;
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
    // relative to mouse in m
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

void TeensyIO::updateMazeState(MouseState &mouseState) {
  GridCoord gc = getGridCoord();
  mouseState.explored[gc.y][gc.x] = true;
  if (std::abs(std::remainder(w.theta, M_PI / 2.0)) > 0.2)
    return;
  if (std::abs(getRotationRate()) > 0.15) {
    return;
  }
  if (!mazeUpdate)
    return;

  if (gc.x < 0 || gc.x >= N || gc.y < 0 || gc.y >= N)
    return;

  WorldCoord rel = w.gridRelativeCoords(gc);

  unsigned char fwdDir, lftDir, rgtDir;
  double fwdPos;
  switch (gc.dir) {
  case TOP:
    fwdDir = TOP;
    lftDir = LEFT;
    rgtDir = RIGHT;
    fwdPos = rel.y;
    break;
  case DOWN:
    fwdDir = DOWN;
    lftDir = RIGHT;
    rgtDir = LEFT;
    fwdPos = CELL_SIZE_METERS - rel.y;
    break;
  case RIGHT:
    fwdDir = RIGHT;
    lftDir = TOP;
    rgtDir = DOWN;
    fwdPos = rel.x;
    break;
  case LEFT:
    fwdDir = LEFT;
    lftDir = DOWN;
    rgtDir = TOP;
    fwdPos = CELL_SIZE_METERS - rel.x;
    break;
  default:
    return;
  }

  double cosT = std::cos(w.theta);
  double sinT = std::sin(w.theta);

  auto addWall = [&](unsigned char wall) {
    mouseState.walls[gc.y][gc.x] |= wall;
    GridCoord adj = dirToVector(wall);
    int nx = gc.x + adj.x;
    int ny = gc.y + adj.y;
    if (nx < 0 || nx >= N || ny < 0 || ny >= N)
      return;

    unsigned char opp;
    switch (wall) {
    case TOP:
      opp = DOWN;
      break;
    case DOWN:
      opp = TOP;
      break;
    case LEFT:
      opp = RIGHT;
      break;
    case RIGHT:
      opp = LEFT;
      break;
    default:
      return;
    }
    mouseState.walls[ny][nx] |= opp;
  };

  if (gc.dir == TOP && rel.y > 0.1)
    return;
  if (gc.dir == DOWN && rel.y < 0.1)
    return;
  if (gc.dir == LEFT && rel.x < 0.1)
    return;
  if (gc.dir == RIGHT && rel.x > 0.1)
    return;
  if (readings[0].y < 0.075) {
    addWall(fwdDir);
  }
  if (-readings[2].x < 0.12) {
    addWall(lftDir);
  }
  if (readings[3].x < 0.11) {
    addWall(rgtDir);
  }
}

void TeensyIO::update(MouseState &mouseState) {
  updateDt();
  updateSensorState();
  updateEncoders();
  updateWorldCoord();
  Serial.printf("COORD: %d, %d  WORLD: %0.2f, %0.2f    WALLS: %d   REL: "
                "%0.2f, %0.2f\n",
                getGridCoord().x, getGridCoord().y, w.x, w.y,
                mouseState.walls[getGridCoord().y][getGridCoord().x],
                w.gridRelativeCoords(getGridCoord()).x,
                w.gridRelativeCoords(getGridCoord()).y);
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
