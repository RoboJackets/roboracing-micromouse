#include "RobotState.h"

double RobotState::distance_to_collision(double x, double y, double theta, bool worldState[240][240]) {
    double dx = std::cos(theta);
    double dy = std::sin(theta);
    double curr_x = x;
    double curr_y = y;
    double distance = 0;
    bool is_collision = false;
    while (distance < 50 && !is_collision) {
        curr_x += dx;
        curr_y += dy;
        int r = std::fmod(curr_x, 1.2);
        int c = std::fmod(curr_y, 1.2);
        if (r < 240 && c < 240 && worldState[r][c]) {
            is_collision = true;
        }
        distance = std::sqrt(std::pow(curr_x - x, 2) + std::pow(curr_y - y, 2));
    }
    return distance;
}

void RobotState::update_state(double left, double right, double delta_t, bool worldState[240][240]) {
    double v_l = ((motor_consts.t * left)/(motor_consts.r * m)) * delta_t;
    double v_r = ((motor_consts.t * right)/(motor_consts.r * m)) * delta_t;
    double r = (l / 2) * ((v_l + v_r) / (v_r - v_l));
    double omega = (v_r - v_l) / l;
    double icc_x = x - r * std::sin(theta);
    double icc_y = y + r * std::cos(theta);

    leftEncoder += v_l * delta_t;
    rightEncoder += v_r * delta_t;

    x = (x - icc_x) * std::cos(omega * delta_t) + (y - icc_y) * -std::sin(omega * delta_t) + icc_x;
    y = (x - icc_x) * std::sin(omega * delta_t) + (y - icc_y) * std::cos(omega * delta_t) + icc_y;
    theta = theta + omega * delta_t;

    for (int i = 0; i < ir_readings.size(); i++) {
        double ir_x = x + ir_consts[i].x_offset;
        double ir_y = y + ir_consts[i].y_offset;
        double ir_angle = theta + ir_consts[i].angle_offset;
        ir_readings[i] = distance_to_collision(ir_x, ir_y, ir_angle, worldState);
    }
}