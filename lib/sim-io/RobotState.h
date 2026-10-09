#pragma once
#include <cmath>
#include <array>


struct IR {
    double x_offset;
    double y_offset;
    double angle_offset;
};

struct Motor {
    double t = .0045; // kg * m (stall torque)
    double r = .016; // m (radius of wheel)
};

struct RobotState {
    // Onboard Electronic Constants
    const Motor motor_consts{};
    const std::array<IR, 4> ir_consts{
        IR{0.00968202, 0.00500721, M_PI / 4},
        IR{0.0130488, 0.00547281, 0},
        IR{0.0130488, 0.00547281, 0},
        IR{0.00818039, 0.0053409, -M_PI / 4}
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
