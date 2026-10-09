#include "StateMachine.h"
#include "Simulator.h"
#include "robot/Robot.h"
#include <iostream>

int main() {
    SimIO simIO{};
    Robot robot{simIO};
    StateMachine mouse{};
    mouse.init(robot);
    robot.setWorldCoord(WorldCoord{0.09, 0.065, M_PI / 2});
    mouse.runFastPath(robot, {Move::Forward, Move::Forward, Move::Right, Move::Forward, Move::Stop});
    while (true) {
        mouse.tick(robot);
        std::cout << "TIME: " << simIO.now() << '\n';

        const WorldCoord w = robot.getWorldCoord();
        std::cout << "WORLD: " << w.x << ", " << w.y << " THETA: "
                  << w.theta << '\n';
        const auto s = robot.getSensorState();
        for (int i = 0; i < 4; ++i) {
            std::cout << i << ": " << s[i].x << ", " << s[i].y << "     ";
        }
        std::cout << '\n';
    }

    return 0;
}
