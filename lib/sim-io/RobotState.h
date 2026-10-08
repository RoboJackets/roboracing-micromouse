#pragma once
#include <cmath>
#include <array>


struct IR {
    double x_offset;
    double y_offset;
    double angle_offset;
};

struct Motor {
    double t = .45; // kg * cm (stall torque)
    double r = 1.6; // cm (radius of wheel)
};

struct RobotState {
    // Onboard Electronic Constants
    const Motor motor_consts{};
    const std::array<IR, 4> ir_consts{
        IR{0.968202, 0.500721, M_PI / 4},
        IR{1.30488, 0.547281, 0},
        IR{1.30488, 0.547281, 0},
        IR{0.818039, 0.53409, -M_PI / 4}
    };

    // Robot Constants
    const double m = .1; // kg (mass)
    const double l = 12; // cm (wheel base of robot model)

    // State Information
    double x;
    double y;
    double theta;
    double leftEncoder;
    double rightEncoder;
    std::array<double, 4> ir_readings;

    double distance_to_collision(double x, double y, double theta, bool worldState[240][240]);
    void update_state(double left, double right, double delta_t, bool worldState[240][240]);
};
