#include "ControlActions.h"

#include <array>

#include "robot/Robot.h"

DriveTimeAction::DriveTimeAction(double time, double speed)
    : finalTime(time), speed(speed) {}

void DriveTimeAction::run(Robot &r) {
  totalTime += r.getDt();
  if (totalTime > finalTime) {
    canceled = true;
    return;
  }
  r.driveDuty(speed, speed);
}

void DriveTimeAction::end(Robot &r) {
  r.driveDuty(0.0, 0.0);
}

YawPIDAction::YawPIDAction(double setpoint) : setpoint(setpoint) {}

void YawPIDAction::run(Robot &r) {
  double measure_r = r.getWorldCoord().theta;
  double error_raw = setpoint - measure_r;
  error = std::atan2(std::sin(error_raw), std::cos(error_raw));

  double avgSpeed =
      0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));
  if (std::abs(error) < YAW_ANGLE_TOL && avgSpeed < VEL_TOL) {
    canceled = true;
    r.driveDuty(0.0, 0.0);
    p.resetAccum();
    return;
  }

  double c = p.calculate(-error, 0, r.getDt());
  r.driveVelocity(-c, c);
}

void YawPIDAction::end(Robot &r) {
  r.driveDuty(0.0, 0.0);
  r.resetPIDs();
  p.resetAccum();
}

ProfiledDriveAction::ProfiledDriveAction(double setpoint, double angle,
                                         double finalVelocity, double maxSpeed)
    : profile({maxSpeed, MAX_ACCEL_M_S2 * DRIVE_ACCEL_SCALE, 0, finalVelocity,
               profilePIDConstants, setpoint}),
      setpoint(setpoint), error(setpoint), angle(angle) {}

void ProfiledDriveAction::run(Robot &r) {
  const std::array<WorldCoord, 4> &ir = r.getSensorState();
  const std::array<WorldCoord, 4> &irAvg = r.getAverageSensorState();
  if (irAvg[0].hypot() < FRONT_WALL_STOP_DISTANCE) {
    profile.finalVelocity = 0;
    canceled = true;
  }
  double avgSpeed =
      0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));
  bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
  if (std::abs(error) < DRIVE_POS_TOL && velOk) {
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
    profile.initialVelocity = vForward * std::cos(w.theta - angle);
    started = true;
  }
  double dx = w.x - prevCoord.x;
  double dy = w.y - prevCoord.y;
  measurement += dx * std::cos(angle) + dy * std::sin(angle);
  prevCoord = w;
  error = setpoint - measurement;
  double v = profile.calculate(r.getDt(), measurement);
  double c = 0;
  double gyroError = angle - w.theta;
  gyroError = std::atan2(std::sin(gyroError), std::cos(gyroError));
  if (std::abs(ir.at(2).x) < IR_VALID_RANGE &&
      std::abs(ir.at(3).x) < IR_VALID_RANGE) {
    c = irPID.calculate(ir.at(3).x, -ir.at(2).x - IR_CENTER_OFFSET, r.getDt());
  } else if (std::abs(ir.at(2).x) < IR_VALID_RANGE) {
    c = irPID.calculate(ir.at(2).x, IR_LEFT_ONLY_SETPOINT, r.getDt());
  } else if (std::abs(ir.at(3).x) < IR_VALID_RANGE) {
    c = irPID.calculate(ir.at(3).x, IR_RIGHT_ONLY_SETPOINT, r.getDt());
  }
  c += gyroPID.calculate(-gyroError, 0, r.getDt());
  r.driveVelocity(v - c, v + c);
}

void ProfiledDriveAction::end(Robot &r) {
  if (profile.finalVelocity == 0) {
    r.driveDuty(0, 0);
  } else {
    r.driveVelocity(profile.finalVelocity, profile.finalVelocity);
  }
}

ProfiledRotationAction::ProfiledRotationAction(double angle)
    : profile({MAX_ROT_SPEED_RAD_S * ROTATION_MAX_SPEED_SCALE,
               MAX_ROT_SPEED_RAD_S2 * ROTATION_ACCEL_SCALE, 0, 0,
               profilePIDConstants, angle}),
      error(angle), setpoint(angle) {}

void ProfiledRotationAction::run(Robot &r) {
  double avgSpeed =
      0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));
  bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
  if (std::abs(error) < ROTATION_POS_TOL && velOk) {
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
  double wheelSpeed = omega * WHEEL_SEPARATION_M / 2;
  r.driveVelocity(-wheelSpeed, wheelSpeed);
}

void ProfiledRotationAction::end(Robot &r) {
  r.driveDuty(0, 0);
}

ProfiledCurveAction::ProfiledCurveAction(double radius, double angle,
                                         double finalVelocity, double maxSpeed)
    : profile({maxSpeed > 0 ? maxSpeed
                            : std::sqrt(COEF_FRICTION * 9.81 * radius) *
                                  CURVE_FRICTION_SPEED_SCALE,
               MAX_ACCEL_M_S2 * CURVE_ACCEL_SCALE, 0, finalVelocity,
               profilePIDConstants, radius * std::abs(angle)}),
      radius(radius), setpoint(radius * angle), error(setpoint) {}

void ProfiledCurveAction::run(Robot &r) {
  double avgSpeed =
      0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));
  bool velOk = (profile.finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
  if (std::abs(error) < CURVE_POS_TOL && velOk) {
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
    profile.initialVelocity = vForward;
    started = true;
  }
  double dTheta = w.theta - prevTheta;
  dTheta = std::atan2(std::sin(dTheta), std::cos(dTheta));
  measurement += dTheta * radius;
  prevTheta = w.theta;
  error = setpoint - measurement;
  double v = profile.calculate(r.getDt(), std::abs(measurement));
  double halfTrack = WHEEL_SEPARATION_M / 2.0;
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

void ProfiledCurveAction::end(Robot &r) {
  if (profile.finalVelocity == 0) {
    r.driveDuty(0, 0);
  } else {
    r.driveVelocity(profile.finalVelocity, profile.finalVelocity);
  }
  irPID.resetAccum();
}
