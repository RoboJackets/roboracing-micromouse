#include "ControlActions.h"

DriveTimeAction::DriveTimeAction(double time, double speed)
    : finalTime(time), speed(speed) {}

void DriveTimeAction::run(MazeMap &map, Robot &r) {
  totalTime += r.getDt();
  if (totalTime > finalTime) {
    canceled = true;
    return;
  }
  r.driveVoltage(1, 1);
}

void DriveTimeAction::end(MazeMap &map, Robot &r) {
  r.driveVoltage(0.0, 0.0);
}

YawPIDAction::YawPIDAction(double setpoint) : setpoint(setpoint) {}

void YawPIDAction::run(MazeMap &map, Robot &r) {
  double measure_r = r.getWorldCoord().theta;
  double error_raw = setpoint - measure_r;
  error = std::atan2(std::sin(error_raw), std::cos(error_raw));

  double avgSpeed =
      0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));
  if (std::abs(error) < 4 * M_PI / 180 && avgSpeed < 0.06) {
    canceled = true;
    r.driveVoltage(0.0, 0.0);
    p.resetAccum();
    return;
  }

  double c = p.calculate(-error, 0, r.getDt());
  r.driveVelocity(-c, c);
}

void YawPIDAction::end(MazeMap &map, Robot &r) {
  r.driveVoltage(0.0, 0.0);
  r.resetPIDs();
  p.resetAccum();
}

SysIDRampAction::SysIDRampAction(double rampRate, double maxTime)
    : rampRate(rampRate), maxTime(maxTime) {}

void SysIDRampAction::run(MazeMap &map, Robot &r) {
  totalTime += r.getDt();
  if (totalTime > maxTime) {
    canceled = true;
    return;
  }
  double voltage = totalTime * rampRate;
  r.driveVoltage(voltage, voltage);

  double speed = (r.getDriveSpeedLeft() + r.getDriveSpeedRight()) / 2.0;
  (void)speed;
}

void SysIDRampAction::end(MazeMap &map, Robot &r) {
  r.driveVoltage(0.0, 0.0);
}

RampVelocityAction::RampVelocityAction(double rampRate, double maxTime)
    : rampRate(rampRate), maxTime(maxTime) {}

void RampVelocityAction::run(MazeMap &map, Robot &r) {
  totalTime += r.getDt();
  if (totalTime > maxTime) {
    canceled = true;
    return;
  }
  double velocity = totalTime * rampRate;
  r.driveVelocity(velocity, velocity);
}

void RampVelocityAction::end(MazeMap &map, Robot &r) {
  r.driveVoltage(0.0, 0.0);
}

ProfiledDriveAction::ProfiledDriveAction(double setpoint, double angle,
                                         double finalVelocity, double maxSpeed)
    : profile({maxSpeed, MAX_ACCEL_M_S2 / 2, 0, finalVelocity,
               profilePIDConstants, setpoint}),
      setpoint(setpoint), error(setpoint), angle(angle) {}

void ProfiledDriveAction::run(MazeMap &map, Robot &r) {
  if (r.getAverageSensorState()[0].hypot() < 0.08) {
    profile.finalVelocity = 0;
    canceled = true;
  }
  double avgSpeed =
      0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));
  bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
  if (std::abs(error) < POS_TOL && velOk) {
    canceled = true;
    return;
  }
  if (canceled) {
    return;
  }
  WorldCoord w = r.getWorldCoord();
  if (!started) {
    prevCoord = w;
    double vForward = (r.getDriveSpeedLeft() + r.getDriveSpeedRight()) / 2.0;
    profile.initalVelocity = vForward * std::cos(w.theta - angle);
    started = true;
  }
  double dx = w.x - prevCoord.x;
  double dy = w.y - prevCoord.y;
  measurement += dx * std::cos(angle) + dy * std::sin(angle);
  prevCoord = w;
  error = setpoint - measurement;
  if ((r.getDriveSpeedLeft() + r.getDriveSpeedRight()) / 2 > best) {
    best = (r.getDriveSpeedLeft() + r.getDriveSpeedRight()) / 2;
  }
  double v = profile.calculate(r.getDt(), measurement);
  double c = 0;
  double gyroError = angle - w.theta;
  gyroError = std::atan2(std::sin(gyroError), std::cos(gyroError));
  if (std::abs(r.getSensorState().at(2).x) < 0.16 &&
      r.getSensorState().at(3).x < 0.16) {
    c = irPID.calculate(r.getSensorState().at(3).x,
                        -r.getSensorState().at(2).x - 0.005, r.getDt());
  } else if (std::abs(r.getSensorState().at(2).x) < 0.16) {
    c = irPID.calculate(r.getSensorState().at(2).x, -0.08, r.getDt());
  } else if (r.getSensorState().at(3).x < 0.16) {
    c = irPID.calculate(r.getSensorState().at(3).x, 0.095, r.getDt());
  }
  c += gyroPID.calculate(-gyroError, 0, r.getDt());
  r.driveVelocity(v - c, v + c);
}

void ProfiledDriveAction::end(MazeMap &map, Robot &r) {
  if (profile.finalVelocity == 0) {
    r.driveVoltage(0, 0);
  } else {
    r.driveVelocity(profile.finalVelocity, profile.finalVelocity);
  }
}

ProfiledRotationAction::ProfiledRotationAction(double angle)
    : profile({MAX_ROT_SPEED_RAD_S * 0.1, MAX_ROT_SPEED_RAD_S2 * 0.5, 0, 0,
               profilePIDConstants, angle}),
      setpoint(angle), error(angle) {}

void ProfiledRotationAction::run(MazeMap &map, Robot &r) {
  double avgSpeed =
      0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));
  bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
  if (std::abs(error) < POS_TOL && velOk) {
    canceled = true;
    return;
  }
  if (canceled) {
    return;
  }
  WorldCoord w = r.getWorldCoord();
  if (!started) {
    prevTheta = w.theta;
    started = true;
  }
  double dTheta = w.theta - prevTheta;
  dTheta = std::atan2(std::sin(dTheta), std::cos(dTheta));
  measurement += dTheta;
  prevTheta = w.theta;
  error = setpoint - measurement;
  double omega = profile.calculate(r.getDt(), measurement);
  double wheelSpeed = omega * WHEEL_SEPERATION_M / 2;
  r.driveVelocity(-wheelSpeed, wheelSpeed);
}

void ProfiledRotationAction::end(MazeMap &map, Robot &r) {
  r.driveVoltage(0, 0);
}

ProfiledCurveAction::ProfiledCurveAction(double radius, double angle,
                                         double finalVelocity, double maxSpeed)
    : profile({maxSpeed > 0 ? maxSpeed
                            : std::sqrt(COEF_FRICTION * 9.81 * radius) * 0.3,
               MAX_ACCEL_M_S2 * 0.5, 0, finalVelocity, profilePIDConstants,
               radius * std::abs(angle)}),
      outerRatio((radius + WHEEL_SEPERATION_M / 2.0) / radius), radius(radius),
      setpoint(radius * angle), error(setpoint) {}

void ProfiledCurveAction::run(MazeMap &map, Robot &r) {
  double avgSpeed =
      0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));
  bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
  if (std::abs(error) < POS_TOL && velOk) {
    canceled = true;
    return;
  }
  if (canceled) {
    return;
  }
  WorldCoord w = r.getWorldCoord();
  if (!started) {
    prevTheta = w.theta;
    double vForward = (r.getDriveSpeedLeft() + r.getDriveSpeedRight()) / 2.0;
    profile.initalVelocity = vForward;
    started = true;
  }
  double dTheta = w.theta - prevTheta;
  dTheta = std::atan2(std::sin(dTheta), std::cos(dTheta));
  measurement += dTheta * radius;
  prevTheta = w.theta;
  error = setpoint - measurement;
  double v = profile.calculate(r.getDt(), std::abs(measurement));
  double halfTrack = WHEEL_SEPERATION_M / 2.0;
  double outerRatio = (radius + halfTrack) / radius;
  double innerRatio = (radius - halfTrack) / radius;
  double c = 0;

  double vOuter = v * outerRatio + c;
  double vInner = v * innerRatio - c;

  if (setpoint > 0) {
    r.driveVelocity(vInner, vOuter);
  } else {
    r.driveVelocity(vOuter, vInner);
  }
}

void ProfiledCurveAction::end(MazeMap &map, Robot &r) {
  if (profile.finalVelocity == 0) {
    r.driveVoltage(0, 0);
  } else {
    r.driveVelocity(profile.finalVelocity, profile.finalVelocity);
  }
  irPID.resetAccum();
}
