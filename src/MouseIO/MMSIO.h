#pragma once
#include "../../mms-cpp/API.h"
#include "MouseIO.h"
#include "IdealState.h"
struct MMSIO : MouseIO {
  int x = 0;
  int y = 0;
  unsigned char dir = TOP;
  GridCoord getGridCoord() override { return GridCoord{x, y}; }
  WorldCoord getWorldCoord() override { return WorldCoord{0.0, 0.0}; }
  void updateWorldCoord() override {}
  unsigned char getGridDir() override { return dir; }

  void drive(double left, double right) override {}
  void getSensorState() override {}
  void update(MouseState& mouseState) override {
    mouseState.x = x;
    mouseState.y = y;
    mouseState.dir = dir;
    mouseState.explored[y][x] = true;
    updateWalls(mouseState);
    logCells(mouseState);
    API::setColor(x,y,'B');
  }
  void logCells(MouseState& state) {
    for (int x = 0; x < N; ++x) {
      for (int y = 0; y < N; ++y) {
        API::setText(x, y, std::to_string(state.dists[y][x]));
      }
    }
  }
  void stateToAPI(IdealState state) {
    GridCoord vector = GridCoord{state.pos.x - x, state.pos.y - y};
    unsigned char newDir = vectorToDir(vector);
    x = state.pos.x;
    y = state.pos.y;
    if (dir == newDir) {
      API::moveForward();
      return;
    }

    unsigned char localBestDir = newDir;
    if (dir == LEFT) localBestDir = RCIRC4(newDir);
    if (dir == RIGHT) localBestDir = LCIRC4(newDir);
    if (dir == DOWN) localBestDir = LCIRC4(LCIRC4(newDir));

    switch (localBestDir) {
      case LEFT:
        API::turnLeft();
        break;
      case RIGHT:
        API::turnRight();
        break;
      case DOWN:
        API::turnLeft();
        API::turnLeft();
        break;
      default:
        break;
    }
    API::moveForward();
    dir = newDir;
  }
  void setState(IdealState state) override {
    if (abs(state.pos.x - x) > 1 || abs(state.pos.y - y) > 1) return;
    stateToAPI(state);
  }
  void updateWalls(MouseState& state) {
    unsigned char localWalls = 0;
    if (API::wallLeft()) localWalls |= LEFT;
    if (API::wallRight()) localWalls |= RIGHT;
    if (API::wallFront()) localWalls |= TOP;

    if (state.dir == LEFT) localWalls = LCIRC4(localWalls);
    if (state.dir == RIGHT) localWalls = RCIRC4(localWalls);
    if (state.dir == DOWN) localWalls = LCIRC4(LCIRC4(localWalls));

    state.walls[state.y][state.x] |= localWalls;

    if (state.x > 0 && (localWalls & LEFT)) {
      API::setWall(state.x, state.y, 'w');
      state.walls[state.y][state.x - 1] |= RIGHT;
    }
    if (state.x < N - 1 && (localWalls & RIGHT)) {
      API::setWall(state.x, state.y, 'e');
      state.walls[state.y][state.x + 1] |= LEFT;
    }
    if (state.y > 0 && (localWalls & DOWN)) {
      API::setWall(state.x, state.y, 's');
      state.walls[state.y - 1][state.x] |= TOP;
    }
    if (state.y < N - 1 && (localWalls & TOP)) {
      API::setWall(state.x, state.y, 'n');
      state.walls[state.y + 1][state.x] |= DOWN;
    }
  }
  void init() override {
    log("Running...");
    API::setColor(0, 0, 'G');
    API::setText(0, 0, "start");
    for (int i = 0; i < N; ++i) {
      API::setWall(0, i, 'w');
      API::setWall(i, 0, 's');
      API::setWall(N - 1, i, 'e');
      API::setWall(i, N - 1, 'n');
    }
  }
};