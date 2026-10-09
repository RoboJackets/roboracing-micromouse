#include "RobotState.h"
#include <limits>

// Distance from (x, y) along heading theta to the first wall square, or infinity
// if there's none within range. Walks the grid squares the ray passes through
// (Amanatides & Woo), so the distance is exact to the face of the square.
double RobotState::distance_to_collision(double x, double y, double theta, bool worldState[240][240]) {
    constexpr double cell = .012;     // m per grid square
    constexpr double max_range = .5;  // m; the sensor reads infinity beyond this
    constexpr double inf = std::numeric_limits<double>::infinity();

    double dx = std::cos(theta);
    double dy = std::sin(theta);
    int r = std::floor(x / cell);
    int c = std::floor(y / cell);
    int step_r = dx > 0 ? 1 : -1;
    int step_c = dy > 0 ? 1 : -1;
    // distance along the ray to the next grid line in x / y, and between lines
    double next_r = dx != 0 ? ((r + (dx > 0)) * cell - x) / dx : inf;
    double next_c = dy != 0 ? ((c + (dy > 0)) * cell - y) / dy : inf;
    double delta_r = dx != 0 ? cell / std::abs(dx) : inf;
    double delta_c = dy != 0 ? cell / std::abs(dy) : inf;

    double distance = 0;
    while (distance <= max_range) {
        if (r < 0 || c < 0 || r >= 240 || c >= 240)
            return inf;
        if (worldState[r][c])
            return distance;
        if (next_r < next_c) {
            distance = next_r;
            next_r += delta_r;
            r += step_r;
        } else {
            distance = next_c;
            next_c += delta_c;
            c += step_c;
        }
    }
    return inf;
}

void RobotState::update_state(double left, double right, double delta_t, bool worldState[240][240]) {
    double v_l = (motor_consts.t * left)/(motor_consts.r * m);
    double v_r = (motor_consts.t * right)/(motor_consts.r * m);
    if (v_l == v_r) {
        x += v_l * std::cos(theta) * delta_t;
        y += v_l * std::sin(theta) * delta_t;
    } else {
        double r = (l / 2) * ((v_l + v_r) / (v_r - v_l));
        double omega = (v_r - v_l) / l;
        // icc = instaneous center of curvature
        double icc_x = x - r * std::sin(theta);
        double icc_y = y + r * std::cos(theta);
    
        double new_x = (x - icc_x) * std::cos(omega * delta_t) + (y - icc_y) * -std::sin(omega * delta_t) + icc_x;
        y = (x - icc_x) * std::sin(omega * delta_t) + (y - icc_y) * std::cos(omega * delta_t) + icc_y;
        x = new_x;
        theta = theta + omega * delta_t;
    }

    leftEncoder += v_l * delta_t;
    rightEncoder += v_r * delta_t;

    for (int i = 0; i < ir_readings.size(); i++) {
        double a = theta - M_PI / 2; // robot frame -> world
        double ir_x = x + ir_consts[i].x_offset * std::cos(a) - ir_consts[i].y_offset * std::sin(a);
        double ir_y = y + ir_consts[i].x_offset * std::sin(a) + ir_consts[i].y_offset * std::cos(a);
        double ir_angle = a + ir_consts[i].angle_offset;
        ir_readings[i] = distance_to_collision(ir_x, ir_y, ir_angle, worldState);
    }
}