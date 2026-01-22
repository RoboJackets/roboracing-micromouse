struct PIDConstants
{
  double p = 0;
  double i = 0;
  double d = 0;
};

class PID
{
  double p = 0;
  double i = 0;
  double d = 0;
  double lastError = 0;
  bool inital = true;
  double lastSetpoint = 0;
  double accum = 0;

public:
  PID(PIDConstants constants) : p(constants.p), i(constants.i), d(constants.d) {}
  double calculate(double measurement, double setpoint, double dt)
  {
    if (inital || lastSetpoint != setpoint)
    {
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
  }
  void resetAccum() { accum = 0; }
};

PIDConstants rot90PID{0.1, 0, 0};