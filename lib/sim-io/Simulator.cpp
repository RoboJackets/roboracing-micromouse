#pragma once
#include "Simulator.h"
#include <array>

void SimIO::init() {
    start_time = SimIO::now();
    current_time = start_time;
}

void SimIO::poll() {
    double now = SimIO::now();
    double delta_t = now - current_time;
    current_time = now;
    robotState.update_state(current_left_pwm, current_right_pwm, delta_t, worldState);
    sampledIr = robotState.ir_readings;
    sampledLeft = robotState.leftEncoder;
    sampledRight = robotState.rightEncoder;
    sampledYaw = robotState.theta;
}

void SimIO::setMotorPwm(double left, double right) {
    current_left_pwm = left;
    current_right_pwm = right;
}

bool SimIO::buttonPressed() {
    // don't actually need to do anything because when the simulator starts we assume the robot wants to start
    // could be made more advanced if a gui is involved, or could read input for elsewhere
    return true;
}

double SimIO::now() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    return millis * 1e-3;
}