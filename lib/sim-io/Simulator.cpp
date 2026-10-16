#pragma once
#include "Simulator.h"
#include <algorithm>
#include <array>
#include <fstream>

void SimIO::addWall(int cx, int cy, Dir d) {
    constexpr int S = 15;
    auto idx = [](int i) { return std::clamp(i, 0, 239); };
    for (int i = 0; i <= S; i++) {
        switch (d) {
        case Dir::North: worldState[idx(cx * S + i)][idx((cy + 1) * S)] = true; break;
        case Dir::South: worldState[idx(cx * S + i)][idx(cy * S)] = true; break;
        case Dir::East:  worldState[idx((cx + 1) * S)][idx(cy * S + i)] = true; break;
        case Dir::West:  worldState[idx(cx * S)][idx(cy * S + i)] = true; break;
        }
    }
}

// load maze from num file (like the one used in mms)
bool SimIO::loadMazeNum(std::ifstream& in) {
    if (!in)
        return false;
    std::fill(&worldState[0][0], &worldState[0][0] + 240 * 240, false);
    int x, y, n, e, s, w;
    while (in >> x >> y >> n >> e >> s >> w) {
        if (n) addWall(x, y, Dir::North);
        if (e) addWall(x, y, Dir::East);
        if (s) addWall(x, y, Dir::South);
        if (w) addWall(x, y, Dir::West);
    }
    return true;
}

void SimIO::printMaze(std::ostream &out) const {
    out << "MAZE: 240 240 ";
    for (int y = 0; y < 240; y++)
        for (int x = 0; x < 240; x++)
            out << (worldState[x][y] ? '1' : '0');
    out << '\n';
}

void SimIO::init() {
    current_time = 0;
}

void SimIO::poll() {
    current_time += dt;
    robotState.update_state(current_left_pwm, current_right_pwm, dt, worldState);
    sampledIr = robotState.ir_readings;
    sampledLeft = robotState.leftEncoder;
    sampledRight = robotState.rightEncoder;
    sampledYaw = -robotState.theta;
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
    return current_time;
}

int SimIO::printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    int result = ::vprintf(format, args);
    va_end(args);
    return result;
}
