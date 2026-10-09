#include "StateMachine.h"
#include "Simulator.h"
#include "robot/Robot.h"
#include <iostream>
#include <fstream>

int main(int argc, char *argv[]) {
    SimIO simIO{};
    Robot robot{simIO};
    StateMachine mouse{};

    if (argc > 1) {
        std::string maze_file_path{ argv[1] };
        if (maze_file_path.ends_with(".num")) {
            std::ifstream in(maze_file_path);
            if (!simIO.loadMazeNum(in)) {
                std::cerr << "failed to load maze from " << maze_file_path << std::endl;
            }
        }
    }

    simIO.printMaze(std::cout);

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
