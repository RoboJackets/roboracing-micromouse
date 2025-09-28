#include <string>

#include "../mms-cpp/API.h"
#include "include/FastPathSolver.h"
#include "include/FloodFillSolver.h"

namespace {
enum class GoalState { GOAL_SEARCH, RETURN, FAST_PATH, NONE };
GoalState currentState = GoalState::GOAL_SEARCH;

MouseState mouseState{};

Solver* solver = nullptr;
const Goals* goal = &CENTER_GOALS;

FloodFillSolver floodFill{};
FastPathSolver fastPath{};
Solver noop = Solver{};

void switchState(GoalState state) {
  if (currentState == state) {
    return;
  }
  solver->onFinished(mouseState, goal);
  switch (currentState) {
    case GoalState::GOAL_SEARCH:
      solver = &floodFill;
      goal = &CENTER_GOALS;
      break;
    case GoalState::RETURN:
      solver = &floodFill;
      goal = &START_GOALS;
      break;
    case GoalState::FAST_PATH:
      solver = &fastPath;
      goal = &CENTER_GOALS;
      break;
    default:
      solver = &noop;
      break;
  }
  solver->init(mouseState, goal);
}
void updateState() {
  switch (currentState) {
    case GoalState::GOAL_SEARCH:
      if (solver->end(mouseState, goal)) currentState = GoalState::RETURN;
      break;
    case GoalState::RETURN:
      if (solver->end(mouseState, goal)) {
        currentState = GoalState::FAST_PATH;
      }
      break;
    default:
      break;
  }
}

void init() {
  mouseState.explored[0][0] = true;
  for (int i = 0; i < CENTER_GOALS.count; ++i) {
    const int gx = CENTER_GOALS.cells[i][1];
    const int gy = CENTER_GOALS.cells[i][0];
    mouseState.explored[gy][gx] = true;
  }
  for (int i = 0; i < N; ++i) {
    mouseState.walls[i][0] |= LEFT;
    mouseState.walls[0][i] |= DOWN;
    mouseState.walls[i][N - 1] |= RIGHT;
    mouseState.walls[N - 1][i] |= TOP;

    API::setWall(0, i, 'w');
    API::setWall(i, 0, 's');
    API::setWall(N - 1, i, 'e');
    API::setWall(i, N - 1, 'n');
  }
  log("Running...");
  API::setColor(0, 0, 'G');
  API::setText(0, 0, "start");
}
}  // namespace

int main() {
  init();

  while (true) {
    updateState();
    solver->run(mouseState, goal);
  }
}
