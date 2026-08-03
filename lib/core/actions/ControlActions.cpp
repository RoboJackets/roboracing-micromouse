#include "ControlActions.h"

DriveTimeAction::DriveTimeAction(double time, double speed)
    : finalTime(time), speed(speed) {}

void DriveTimeAction::run(MouseState &s, MouseIO &io) {
  totalTime += io.getDt();
  if (totalTime > finalTime) {
    canceled = true;
    return;
  }
  io.driveVoltage(1, 1);
}

void DriveTimeAction::end(MouseState &s, MouseIO &io) {
  io.driveVoltage(0.0, 0.0);
}

YawPIDAction::YawPIDAction(double setpoint) : setpoint(setpoint) {}

void YawPIDAction::run(MouseState &s, MouseIO &io) {
  double measure_r = io.getWorldCoord().theta;
  double error_raw = setpoint - measure_r;
  error = std::atan2(std::sin(error_raw), std::cos(error_raw));

  double avgSpeed =
      0.5 * (std::abs(io.getDriveSpeedLeft()) + std::abs(io.getDriveSpeedRight()));
  if (std::abs(error) < 4 * M_PI / 180 && avgSpeed < 0.06) {
    canceled = true;
    io.driveVoltage(0.0, 0.0);
    p.resetAccum();
    return;
  }

  double c = p.calculate(-error, 0, io.getDt());
  io.driveVelocity(-c, c);
}

void YawPIDAction::end(MouseState &s, MouseIO &io) {
  io.driveVoltage(0.0, 0.0);
  io.resetPIDs();
  p.resetAccum();
}

SysIDRampAction::SysIDRampAction(double rampRate, double maxTime)
    : rampRate(rampRate), maxTime(maxTime) {}

void SysIDRampAction::run(MouseState &s, MouseIO &io) {
  totalTime += io.getDt();
  if (totalTime > maxTime) {
    canceled = true;
    return;
  }
  double voltage = totalTime * rampRate;
  io.driveVoltage(voltage, voltage);

  double speed = (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2.0;
  (void)speed;
}

void SysIDRampAction::end(MouseState &s, MouseIO &io) {
  io.driveVoltage(0.0, 0.0);
}

RampVelocityAction::RampVelocityAction(double rampRate, double maxTime)
    : rampRate(rampRate), maxTime(maxTime) {}

void RampVelocityAction::run(MouseState &s, MouseIO &io) {
  totalTime += io.getDt();
  if (totalTime > maxTime) {
    canceled = true;
    return;
  }
  double velocity = totalTime * rampRate;
  io.driveVelocity(velocity, velocity);
}

void RampVelocityAction::end(MouseState &s, MouseIO &io) {
  io.driveVoltage(0.0, 0.0);
}

ProfiledDriveAction::ProfiledDriveAction(double setpoint, double angle,
                                         double finalVelocity, double maxSpeed)
    : profile({maxSpeed, MAX_ACCEL_M_S2 / 2, 0, finalVelocity,
               profilePIDConstants, setpoint}),
      setpoint(setpoint), error(setpoint), angle(angle) {}

void ProfiledDriveAction::run(MouseState &s, MouseIO &io) {
  if (io.getAverageSensorState()[0].hypot() < 0.08) {
    profile.finalVelocity = 0;
    canceled = true;
  }
  double avgSpeed =
      0.5 * (std::abs(io.getDriveSpeedLeft()) + std::abs(io.getDriveSpeedRight()));
  bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
  if (std::abs(error) < POS_TOL && velOk) {
    canceled = true;
    return;
  }
  if (canceled) {
    return;
  }
  WorldCoord w = io.getWorldCoord();
  if (!started) {
    prevCoord = w;
    double vForward = (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2.0;
    profile.initalVelocity = vForward * std::cos(w.theta - angle);
    started = true;
  }
  double dx = w.x - prevCoord.x;
  double dy = w.y - prevCoord.y;
  measurement += dx * std::cos(angle) + dy * std::sin(angle);
  prevCoord = w;
  error = setpoint - measurement;
  if ((io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2 > best) {
    best = (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2;
  }
  double v = profile.calculate(io.getDt(), measurement);
  double c = 0;
  double gyroError = angle - w.theta;
  gyroError = std::atan2(std::sin(gyroError), std::cos(gyroError));
  if (std::abs(io.getSensorState().at(2).x) < 0.16 &&
      io.getSensorState().at(3).x < 0.16) {
    c = irPID.calculate(io.getSensorState().at(3).x,
                        -io.getSensorState().at(2).x - 0.005, io.getDt());
  } else if (std::abs(io.getSensorState().at(2).x) < 0.16) {
    c = irPID.calculate(io.getSensorState().at(2).x, -0.08, io.getDt());
  } else if (io.getSensorState().at(3).x < 0.16) {
    c = irPID.calculate(io.getSensorState().at(3).x, 0.095, io.getDt());
  }
  c += gyroPID.calculate(-gyroError, 0, io.getDt());
  io.driveVelocity(v - c, v + c);
}

void ProfiledDriveAction::end(MouseState &s, MouseIO &io) {
  if (profile.finalVelocity == 0) {
    io.driveVoltage(0, 0);
  } else {
    io.driveVelocity(profile.finalVelocity, profile.finalVelocity);
  }
}

ProfiledRotationAction::ProfiledRotationAction(double angle)
    : profile({MAX_ROT_SPEED_RAD_S * 0.1, MAX_ROT_SPEED_RAD_S2 * 0.5, 0, 0,
               profilePIDConstants, angle}),
      setpoint(angle), error(angle) {}

void ProfiledRotationAction::run(MouseState &s, MouseIO &io) {
  double avgSpeed =
      0.5 * (std::abs(io.getDriveSpeedLeft()) + std::abs(io.getDriveSpeedRight()));
  bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
  if (std::abs(error) < POS_TOL && velOk) {
    canceled = true;
    return;
  }
  if (canceled) {
    return;
  }
  WorldCoord w = io.getWorldCoord();
  if (!started) {
    prevTheta = w.theta;
    started = true;
  }
  double dTheta = w.theta - prevTheta;
  dTheta = std::atan2(std::sin(dTheta), std::cos(dTheta));
  measurement += dTheta;
  prevTheta = w.theta;
  error = setpoint - measurement;
  double omega = profile.calculate(io.getDt(), measurement);
  double wheelSpeed = omega * WHEEL_SEPERATION_M / 2;
  io.driveVelocity(-wheelSpeed, wheelSpeed);
}

void ProfiledRotationAction::end(MouseState &s, MouseIO &io) {
  io.driveVoltage(0, 0);
}

ProfiledCurveAction::ProfiledCurveAction(double radius, double angle,
                                         double finalVelocity, double maxSpeed)
    : profile({maxSpeed > 0 ? maxSpeed
                            : std::sqrt(COEF_FRICTION * 9.81 * radius) * 0.3,
               MAX_ACCEL_M_S2 * 0.5, 0, finalVelocity, profilePIDConstants,
               radius * std::abs(angle)}),
      outerRatio((radius + WHEEL_SEPERATION_M / 2.0) / radius), radius(radius),
      setpoint(radius * angle), error(setpoint) {}

void ProfiledCurveAction::run(MouseState &s, MouseIO &io) {
  double avgSpeed =
      0.5 * (std::abs(io.getDriveSpeedLeft()) + std::abs(io.getDriveSpeedRight()));
  bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
  if (std::abs(error) < POS_TOL && velOk) {
    canceled = true;
    return;
  }
  if (canceled) {
    return;
  }
  WorldCoord w = io.getWorldCoord();
  if (!started) {
    prevTheta = w.theta;
    double vForward = (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) / 2.0;
    profile.initalVelocity = vForward;
    started = true;
  }
  double dTheta = w.theta - prevTheta;
  dTheta = std::atan2(std::sin(dTheta), std::cos(dTheta));
  measurement += dTheta * radius;
  prevTheta = w.theta;
  error = setpoint - measurement;
  double v = profile.calculate(io.getDt(), std::abs(measurement));
  double halfTrack = WHEEL_SEPERATION_M / 2.0;
  double outerRatio = (radius + halfTrack) / radius;
  double innerRatio = (radius - halfTrack) / radius;
  double c = 0;

  double vOuter = v * outerRatio + c;
  double vInner = v * innerRatio - c;

  if (setpoint > 0) {
    io.driveVelocity(vInner, vOuter);
  } else {
    io.driveVelocity(vOuter, vInner);
  }
}

void ProfiledCurveAction::end(MouseState &s, MouseIO &io) {
  if (profile.finalVelocity == 0) {
    io.driveVoltage(0, 0);
  } else {
    io.driveVelocity(profile.finalVelocity, profile.finalVelocity);
  }
  irPID.resetAccum();
}
