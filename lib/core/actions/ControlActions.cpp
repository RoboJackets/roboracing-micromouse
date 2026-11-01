#include "ControlActions.h"

#include <array>

#include "robot/Robot.h"

void StopAction::run(Robot &r) {
  r.driveDuty(0.0, 0.0);
  canceled = true;
}

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
  double measure_r = r.getHeading();
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

ProfiledDriveAction::ProfiledDriveAction(WorldCoord begin, WorldCoord target,
                                         TrapezoidalProfile::Constraints constraints)
    : profile{constraints},
      begin{begin},
      target{target},
      dir{(target - begin).normalized()},
      lineAngle{std::atan2(dir.y, dir.x)}
{
}

void ProfiledDriveAction::run(Robot &r) {
  double dt = r.getDt();
  WorldCoord w = r.getWorldCoord();
  double dist = (target - begin).hypot(); // scalar distance to target
  double measuredS = (w - begin).dot(dir); // component of robot "progress" that lies on dir
  double avgSpeed = 0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));

  r.io.printf("[drive]\n");
  const std::array<WorldCoord, 4> &ir = r.getSensorState();
  const std::array<WorldCoord, 4> &irAvg = r.getAverageSensorState();
  // if (irAvg[0].hypot() < FRONT_WALL_STOP_DISTANCE) {
  //   profile.finalVelocity = 0;
  //   canceled = true;
  //   r.io.printf("canceled\n");
  // }
  
  bool distOk = std::abs(dist - measuredS) < DRIVE_POS_TOL;
  bool velOk = avgSpeed < VEL_TOL; // TODO: nonzero target velocity
  if (distOk && velOk) {
    canceled = true;
    return;
  }
  if (canceled) {
    return;
  }

  TrapezoidalProfile::State ref = profile.calculate(time, {0, 0}, {dist, 0});

  // trapezoidal profile output velocity, plus PID velocity to
  // get robot measuredS closer to trapezoidal profile target
  double v = ref.velocity + distPID.calculate(measuredS, ref.position, dt);

  double c = 0;
  double gyroError = std::remainder(lineAngle - r.getHeading(), 2 * M_PI);

  // if (std::abs(ir.at(2).x) < IR_VALID_RANGE &&
  //     std::abs(ir.at(3).x) < IR_VALID_RANGE) {
  //   c = irPID.calculate(ir.at(3).x, -ir.at(2).x - IR_CENTER_OFFSET, r.getDt());
  // } else if (std::abs(ir.at(2).x) < IR_VALID_RANGE) {
  //   c = irPID.calculate(ir.at(2).x, IR_LEFT_ONLY_SETPOINT, r.getDt());
  // } else if (std::abs(ir.at(3).x) < IR_VALID_RANGE) {
  //   c = irPID.calculate(ir.at(3).x, IR_RIGHT_ONLY_SETPOINT, r.getDt());
  // }
  c += gyroPID.calculate(-gyroError, 0, r.getDt());
  r.driveVelocity(v - c, v + c);
}

void ProfiledDriveAction::end(Robot &r) {
  r.driveDuty(0, 0);
  // if (profile.finalVelocity == 0) {
  // } else {
  //   r.driveVelocity(profile.finalVelocity, profile.finalVelocity);
  // }
}

ProfiledRotationAction::ProfiledRotationAction(double angle)
    : profile({MAX_ROT_SPEED_RAD_S * ROTATION_MAX_SPEED_SCALE,
               MAX_ROT_SPEED_RAD_S2 * ROTATION_ACCEL_SCALE}),
      error(angle), setpoint(angle) {}

void ProfiledRotationAction::run(Robot &r) {
  double avgSpeed =
      0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));
  bool velOk = avgSpeed < VEL_TOL;
  double theta = r.getHeading();
  r.io.printf("error: %f\tavgSpeed: %f\n", error, avgSpeed);
  if (std::abs(error) < ROTATION_POS_TOL && velOk) {
    canceled = true;
    return;
  }
  if (canceled) {
    return;
  }
  WorldCoord w = r.getWorldCoord();
  if (!started) {
    prevTheta = theta;
    started = true;
  }
  double dTheta = theta - prevTheta;
  dTheta = std::atan2(std::sin(dTheta), std::cos(dTheta));
  measurement += dTheta;
  prevTheta = theta;
  error = setpoint - measurement;
  profileSetpoint =
      profile.calculate(r.getDt(), profileSetpoint, {setpoint, 0});
  double omega =
      profileSetpoint.velocity +
      errorPid.calculate(measurement, profileSetpoint.position, r.getDt());
  omega = std::clamp(omega, -profile.cts.maxSpeed, profile.cts.maxSpeed);
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
               MAX_ACCEL_M_S2 * CURVE_ACCEL_SCALE}),
      finalVelocity(finalVelocity), radius(radius), setpoint(radius * angle),
      error(setpoint) {}

void ProfiledCurveAction::run(Robot &r) {
  double theta = r.getHeading();
  double avgSpeed =
      0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));
  bool velOk = (finalVelocity == 0) ? (avgSpeed < VEL_TOL) : true;
  r.io.printf("error: %f\tavgSpeed: %f\n", error, avgSpeed);
  if (std::abs(error) < CURVE_POS_TOL && velOk) {
    canceled = true;
    return;
  }
  if (canceled) {
    return;
  }
  WorldCoord w = r.getWorldCoord();
  if (!started) {
    prevTheta = theta;
    double vForward = (r.getDriveSpeedLeft() + r.getDriveSpeedRight()) / 2.0;
    profileSetpoint = {0, vForward};
    started = true;
  }
  double dTheta = theta - prevTheta;
  dTheta = std::atan2(std::sin(dTheta), std::cos(dTheta));
  measurement += dTheta * radius;
  prevTheta = theta;
  error = setpoint - measurement;
  profileSetpoint = profile.calculate(r.getDt(), profileSetpoint,
                                      {std::abs(setpoint), finalVelocity});
  double v = profileSetpoint.velocity +
             errorPid.calculate(std::abs(measurement), profileSetpoint.position,
                                r.getDt());
  v = std::clamp(v, -profile.cts.maxSpeed, profile.cts.maxSpeed);
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
  if (finalVelocity == 0) {
    r.driveDuty(0, 0);
  } else {
    r.driveVelocity(finalVelocity, finalVelocity);
  }
  irPID.resetAccum();
}
