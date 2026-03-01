#pragma once
#include <Action.h>
#include <cmath>
#include <tuple>
#include <vector>

struct PIDConstants {
  double p = 0;
  double i = 0;
  double d = 0;
};

inline PIDConstants rot90PIDConstants{0.01, 0, 0};
inline PIDConstants velocityPIDConstants{0, 0, 0};
inline PIDConstants profilePIDConstants{0, 0, 0};

class PID {
  double p = 0;
  double i = 0;
  double d = 0;
  double lastError = 0;
  bool inital = true;
  double lastSetpoint = 0;
  double accum = 0;

public:
  PID(PIDConstants constants)
      : p(constants.p), i(constants.i), d(constants.d) {}
  double calculate(double measurement, double setpoint, double dt) {
    if (inital || lastSetpoint != setpoint) {
      lastSetpoint = setpoint;
      inital = false;
      lastError = setpoint - measurement;
    }
    double error = setpoint - measurement;
    double result = 0;
    result += measurement * p;
    double derivative = (error - lastError) / dt;
    result += derivative * d;
    accum += error * dt;
    result += i * accum;
    lastError = error;
    return result;
  }
  void resetAccum() { accum = 0; }
};

struct TrapezoidalProfile {
  double maxSpeed, maxAccel, initalVelocity, finalVelocity;
  PIDConstants pidConstants;
  double setpoint;
  double time = 0;
  PID errorPid = PID{pidConstants};
  TrapezoidalProfile(double maxSpeed, double maxAccel, double initalVelocity,
                     double finalVelocity, PIDConstants pidConstants,
                     double setpoint)
      : maxSpeed(maxSpeed), maxAccel(maxAccel), initalVelocity(initalVelocity),
        finalVelocity(finalVelocity), pidConstants(pidConstants),
        setpoint(setpoint) {}

  double calculate(double dt, double measurement) {
    time += dt;

    double error = setpoint - measurement;
    double direction = error >= 0.0 ? 1.0 : -1.0;

    double D = std::abs(error);
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

    return velocity +
           errorPid.calculate(measurement, measurement + position, dt);
  }

  void reset() {
    time = 0;
    errorPid.resetAccum();
  }

  static double totalTime(double maxAccel, double maxSpeed, double distance) {
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
};

struct MotorFeedForward {
  double ks;
  double kv;
  double ka;
  double lastVelocity = 0;

  MotorFeedForward(double ks, double kv, double ka) : ks(ks), kv(kv), ka(ka) {}

  double calculate(double velocitySetpoint, double dt) {
    double sign = (0 < velocitySetpoint) - (velocitySetpoint < 0);
    double voltage = ks * sign + kv * velocitySetpoint +
                     ka * (velocitySetpoint - lastVelocity) / dt;
    lastVelocity = velocitySetpoint;
    return voltage;
  }
};

// struct SysIDAction : Action {
//   bool canceled = false;
//   double quasistaticTime, dynamicTime;
//   double voltAccel, voltDynamic;

//   std::vector<std::tuple<double, double>> quasiForward = {};
//   std::vector<std::tuple<double, double>> quasiBackward = {};
//   std::vector<std::tuple<double, double>> dynamicForward = {};
//   std::vector<std::tuple<double, double>> dynamicBackward = {};

//   double totalTime = 0;

//   SysIDAction(double quasistaticTime, double dynamicTime)
//       : quasistaticTime(quasistaticTime), dynamicTime(dynamicTime) {}
//   void cancel() override { canceled = true; }
//   bool completed() const override { return canceled; }
//   void run(MouseState &s, MouseIO &io) override {
//     double t0 = totalTime + quasistaticTime;
//     double t1 = t0 + 3;
//     double t2 = t1 + quasistaticTime;
//     double t3 = t2 + 3;
//     double t4 = t3 + dynamicTime;
//     double t5 = t4 + 3;
//     double t6 = t5 + dynamicTime;
//     if (totalTime <= t0) {
//       quasiForward.push_back(
//           {totalTime, (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) /
//           2});
//       io.driveVoltage(totalTime * voltAccel, totalTime * voltAccel);
//     } else if (totalTime >= t1 && totalTime <= t2) {
//       quasiBackward.push_back(
//           {totalTime, (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) /
//           2});
//       io.driveVoltage(-(totalTime - t1) * voltAccel,
//                       -(totalTime - t1) * voltAccel);
//     } else if (totalTime >= t3 && totalTime <= t4) {
//       dynamicForward.push_back(
//           {totalTime, (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) /
//           2});
//       io.driveVoltage(voltDynamic, voltDynamic);
//     } else if (totalTime >= t5 && totalTime <= t6) {
//       dynamicBackward.push_back(
//           {totalTime, (io.getDriveSpeedLeft() + io.getDriveSpeedRight()) /
//           2});
//       io.driveVoltage(-voltDynamic, -voltDynamic);
//     } else if (totalTime > t6 + 1) {
//       cancel();
//     } else {
//       io.driveVoltage(0, 0);
//     }
//     totalTime += io.getDt();
//   }
//   void end(MouseState &s, MouseIO &io) override { io.driveVoltage(0.0, 0.0);
//   }
// };