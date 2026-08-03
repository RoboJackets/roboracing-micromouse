#include "ControlAlgorithms.h"

PID::PID(PIDConstants constants)
    : p(constants.p), i(constants.i), d(constants.d),
      maxAccum(constants.maxAccum) {}

double PID::calculate(double measurement, double setpoint, double dt) {
  if (inital || lastSetpoint != setpoint) {
    lastSetpoint = setpoint;
    inital = false;
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

TrapezoidalProfile::TrapezoidalProfile(double maxSpeed, double maxAccel,
                                       double initalVelocity,
                                       double finalVelocity,
                                       PIDConstants pidConstants,
                                       double setpoint)
    : maxSpeed(maxSpeed), maxAccel(maxAccel), initalVelocity(initalVelocity),
      finalVelocity(finalVelocity), pidConstants(pidConstants),
      setpoint(setpoint) {}

double TrapezoidalProfile::calculate(double dt, double measurement) {
  if (!started) {
    startMeasurement = measurement;
    started = true;
  }
  time += dt;

  double totalDistance = setpoint - startMeasurement;
  double direction = totalDistance >= 0.0 ? 1.0 : -1.0;

  double D = std::abs(totalDistance);
  double v0 = std::abs(initalVelocity);
  double vf = std::abs(finalVelocity);

  v0 = std::min(v0, maxSpeed);
  vf = std::min(vf, maxSpeed);

  double tA = (maxSpeed - v0) / maxAccel;
  double tD = (maxSpeed - vf) / maxAccel;

  tA = std::max(0.0, tA);
  tD = std::max(0.0, tD);

  double dA = (v0 + maxSpeed) * 0.5 * tA;
  double dD = (vf + maxSpeed) * 0.5 * tD;

  double velocity = 0.0;
  double position = 0.0;

  if (D >= dA + dD) {
    double dC = D - dA - dD;
    double tC = dC / maxSpeed;
    double T = tA + tC + tD;

    if (time < tA) {
      velocity = v0 + maxAccel * time;
      position = v0 * time + 0.5 * maxAccel * time * time;
    } else if (time < tA + tC) {
      double t = time - tA;
      velocity = maxSpeed;
      position = dA + maxSpeed * t;
    } else if (time < T) {
      double t = time - tA - tC;
      velocity = maxSpeed - maxAccel * t;
      position = dA + dC + maxSpeed * t - 0.5 * maxAccel * t * t;
    } else {
      velocity = vf;
      position = D;
    }
  } else {
    double vp2 = maxAccel * D + 0.5 * (v0 * v0 + vf * vf);
    double vp = std::sqrt(std::max(0.0, vp2));

    double tA_tri = (vp - v0) / maxAccel;
    double tD_tri = (vp - vf) / maxAccel;
    double T = tA_tri + tD_tri;

    tA_tri = std::max(0.0, tA_tri);
    tD_tri = std::max(0.0, tD_tri);

    double dA_tri = (v0 + vp) * 0.5 * tA_tri;

    if (time < tA_tri) {
      velocity = v0 + maxAccel * time;
      position = v0 * time + 0.5 * maxAccel * time * time;
    } else if (time < T) {
      double t = time - tA_tri;
      velocity = vp - maxAccel * t;
      position = dA_tri + vp * t - 0.5 * maxAccel * t * t;
    } else {
      velocity = vf;
      position = D;
    }
  }

  velocity *= direction;
  position *= direction;

  double desiredPosition = startMeasurement + position;
  double output =
      velocity + errorPid.calculate(measurement, desiredPosition, dt);
  return std::clamp(output, -maxSpeed, maxSpeed);
}

void TrapezoidalProfile::reset() {
  time = 0;
  started = false;
  errorPid.resetAccum();
}

double TrapezoidalProfile::totalTime(double maxAccel, double maxSpeed,
                                     double distance) {
  double D = std::abs(distance);

  double accelTime = maxSpeed / maxAccel;
  double accelDistance = 0.5 * maxAccel * accelTime * accelTime;

  if (D >= 2.0 * accelDistance) {
    double cruiseTime = (D - 2.0 * accelDistance) / maxSpeed;
    return 2.0 * accelTime + cruiseTime;
  } else {
    double peakVelocity = std::sqrt(D * maxAccel);
    double peakTime = peakVelocity / maxAccel;
    return 2.0 * peakTime;
  }
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
