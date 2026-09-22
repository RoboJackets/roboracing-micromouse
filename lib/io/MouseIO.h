#pragma once
#include <array>

struct MouseIO {
  virtual ~MouseIO() = default;

  /**
  * @brief Initial setup for IO. Typically will include seting up pinouts and members for the IO instance.
  */
  virtual void init() = 0;

  /**
   * @brief Updates the members that store information about current sensor state.
   */
  virtual void poll() = 0;

  /**
   * @brief Set motor PWM, which in this case refers to setting the speed of the motors.
   * 
   * @param left floating point number between 1.0 and -1.0; positive is forward robot relative
   * @param right floating point number between 1.0 and -1.0; positive is forward robot relative
   */
  virtual void setMotorPwm(double left, double right) = 0;

  /**
   * @brief Returns the lastest left motor encoder reading in meters.
   * 
   * @return double left motor encoder reading in meters
   */
  virtual double leftMeters() const = 0;

  /**
   * @brief Returns the latest right motor encoder reading in meters.
   * 
   * @return double right motor encoder reading in meters
   */
  virtual double rightMeters() const = 0;

  /**
   * @brief Returns the latest gyro yaw in radians.
   * 
   * @return double latest gyro yaw in radians
   */
  virtual double gyroYaw() const = 0;

  /**
   * @brief Returns array of latest IR sensor readings. The convention is sensors are
   * added in the array such that the left most sensor is at index 0 and the rightmost sensor
   * is at index irMeters.size().
   * 
   * The array lenght should be a variable: https://github.com/RoboJackets/roboracing-micromouse/issues/52
   * 
   * @return const std::array<double, 4>& Array of latest sensor readings
   */
  virtual const std::array<double, 4> &irMeters() const = 0;

  /**
   * @brief This button is the only way to provide input to the mouse. Usually used for startup and changing mode.
   * 
   * @return true button is pressed
   * @return false button is not pressed
   */
  virtual bool buttonPressed() = 0;
  
  /**
   * @brief Returns the time since robot start.
   * 
   * @return double time since robot start
   */
  virtual double now() = 0;
};
