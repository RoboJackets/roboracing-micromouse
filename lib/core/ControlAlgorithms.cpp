#include "ControlAlgorithms.h"

PID::PID(PIDConstants constants)
    : p(constants.p), i(constants.i), d(constants.d),
      maxAccum(constants.maxAccum) {}

double PID::calculate(double measurement, double setpoint, double dt) {
  if (initial || lastSetpoint != setpoint) {
    lastSetpoint = setpoint;
    initial = false;
    lastError = setpoint - measurement;
  }
  double error = setpoint - measurement;
  double result = 0;
  result += error * p;
  double derivative = (error - lastError) / dt;
  result += derivative * d;
  accum += error * dt;
  accum = std::clamp(accum, -maxAccum, maxAccum);
  result += i * accum;
  lastError = error;
  return result;
}

TrapezoidalProfile::TrapezoidalProfile(KinematicConstraints constraints)
    : cts{constraints} {}

KinematicState TrapezoidalProfile::calculate(double dt,
                                                        KinematicState setpoint,
                                                        KinematicState goal) const {
  double totalDistance = goal.position - setpoint.position;
  double direction = totalDistance >= 0.0 ? 1.0 : -1.0;

  double D = std::abs(totalDistance);
  double v0 = setpoint.velocity * direction;
  double vf = std::abs(goal.velocity);

  v0 = std::min(v0, cts.maxSpeed);
  vf = std::min(vf, cts.maxSpeed);

  double velocity = 0.0;
  double position = 0.0;

  if (v0 < 0.0 || v0 * v0 - vf * vf > 2.0 * cts.maxAccel * D) {
    // moving away, or too fast to slow to vf in time: brake at max accel
    double accel = v0 < 0.0 ? cts.maxAccel : -cts.maxAccel;
    velocity = v0 + accel * dt;
    position = v0 * dt + 0.5 * accel * dt * dt;
    return {setpoint.position + position * direction, velocity * direction};
  }

  double tA = (cts.maxSpeed - v0) / cts.maxAccel;
  double tD = (cts.maxSpeed - vf) / cts.maxAccel;

  tA = std::max(0.0, tA);
  tD = std::max(0.0, tD);

  double dA = (v0 + cts.maxSpeed) * 0.5 * tA;
  double dD = (vf + cts.maxSpeed) * 0.5 * tD;

  if (D >= dA + dD) {
    double dC = D - dA - dD;
    double tC = dC / cts.maxSpeed;
    double T = tA + tC + tD;

    if (dt < tA) {
      velocity = v0 + cts.maxAccel * dt;
      position = v0 * dt + 0.5 * cts.maxAccel * dt * dt;
    } else if (dt < tA + tC) {
      double t = dt - tA;
      velocity = cts.maxSpeed;
      position = dA + cts.maxSpeed * t;
    } else if (dt < T) {
      double t = dt - tA - tC;
      velocity = cts.maxSpeed - cts.maxAccel * t;
      position = dA + dC + cts.maxSpeed * t - 0.5 * cts.maxAccel * t * t;
    } else {
      velocity = vf;
      position = D;
    }
  } else {
    double vp2 = cts.maxAccel * D + 0.5 * (v0 * v0 + vf * vf);
    double vp = std::sqrt(std::max(0.0, vp2));

    double tA_tri = (vp - v0) / cts.maxAccel;
    double tD_tri = (vp - vf) / cts.maxAccel;

    tA_tri = std::max(0.0, tA_tri);
    tD_tri = std::max(0.0, tD_tri);

    double T = tA_tri + tD_tri;

    double dA_tri = (v0 + vp) * 0.5 * tA_tri;

    if (dt < tA_tri) {
      velocity = v0 + cts.maxAccel * dt;
      position = v0 * dt + 0.5 * cts.maxAccel * dt * dt;
    } else if (dt < T) {
      double t = dt - tA_tri;
      velocity = vp - cts.maxAccel * t;
      position = dA_tri + vp * t - 0.5 * cts.maxAccel * t * t;
    } else {
      velocity = vf;
      position = D;
    }
  }

  return {setpoint.position + position * direction, velocity * direction};
}

double TrapezoidalProfile::totalTime(KinematicState setpoint, KinematicState goal) const {
  double totalDistance = goal.position - setpoint.position;
  double direction = totalDistance >= 0.0 ? 1.0 : -1.0;

  double D = std::abs(totalDistance);
  double v0 = setpoint.velocity * direction;
  double vf = std::abs(goal.velocity);

  v0 = std::min(v0, cts.maxSpeed);
  vf = std::min(vf, cts.maxSpeed);

  if (v0 < 0.0 || v0 * v0 - vf * vf > 2.0 * cts.maxAccel * D) {
    // calculate() brakes to a stop here, then profiles from rest
    double tStop = std::abs(v0) / cts.maxAccel;
    double stopPosition = setpoint.position +
                          direction * v0 * std::abs(v0) / (2.0 * cts.maxAccel);
    return tStop + totalTime({stopPosition, 0.0}, goal);
  }

  double tA = std::max(0.0, (cts.maxSpeed - v0) / cts.maxAccel);
  double tD = std::max(0.0, (cts.maxSpeed - vf) / cts.maxAccel);

  double dA = (v0 + cts.maxSpeed) * 0.5 * tA;
  double dD = (vf + cts.maxSpeed) * 0.5 * tD;

  if (D >= dA + dD) {
    double tC = (D - dA - dD) / cts.maxSpeed;
    return tA + tC + tD;
  }

  double vp2 = cts.maxAccel * D + 0.5 * (v0 * v0 + vf * vf);
  double vp = std::sqrt(std::max(0.0, vp2));

  double tA_tri = std::max(0.0, (vp - v0) / cts.maxAccel);
  double tD_tri = std::max(0.0, (vp - vf) / cts.maxAccel);
  return tA_tri + tD_tri;
}

MotorFeedForward::MotorFeedForward(double ks, double kv, double ka)
    : ks(ks), kv(kv), ka(ka) {}

double MotorFeedForward::calculate(double velocitySetpoint, double dt) {
  double sign = (0 < velocitySetpoint) - (velocitySetpoint < 0);
  double voltage = ks * sign + kv * velocitySetpoint +
                   ka * (velocitySetpoint - lastVelocity) / dt;
  lastVelocity = velocitySetpoint;
  return voltage;
}
