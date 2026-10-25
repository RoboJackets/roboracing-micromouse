#pragma once
#include <cmath>
#include <array>
#include "robot/DistanceSensors.h"


struct IR {
    double x_offset;
    double y_offset;
    double angle_offset;
};

struct Motor {
    double t = .00229; // kg * m (stall torque)
    double r = .016; // m (radius of wheel)
    double static_duty = .4; // duty lost to static friction
};

struct RobotState {
    // Onboard Electronic Constants
    const Motor motor_consts{};
    // Same mounts the robot uses: robot frame, +y forward, +x right
    const std::array<IR, 4> ir_consts{
        IR{IR_MOUNTS[0].x, IR_MOUNTS[0].y, IR_MOUNTS[0].theta},
        IR{IR_MOUNTS[1].x, IR_MOUNTS[1].y, IR_MOUNTS[1].theta},
        IR{IR_MOUNTS[2].x, IR_MOUNTS[2].y, IR_MOUNTS[2].theta},
        IR{IR_MOUNTS[3].x, IR_MOUNTS[3].y, IR_MOUNTS[3].theta}
    };

    // Robot Constants
    const double m = .1; // kg (mass)
    const double l = .12; // m (wheel base of robot model)

    // State Information
    double x = .09;
    double y = .065;
    double theta = M_PI / 2;
    double leftEncoder = 0;
    double rightEncoder = 0;
    std::array<double, 4> ir_readings{};

    double distance_to_collision(double x, double y, double theta, bool worldState[240][240]);
    void update_state(double left, double right, double delta_t, bool worldState[240][240]);
};
