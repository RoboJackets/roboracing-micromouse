#include <string>

#include "../mms-cpp/API.h"
#include "IO/MMSIO.h"
#include "include/FastPathSolver.h"
#include "include/FloodFillSolver.h"
#include "include/CellSelection.h"

namespace {
enum class GoalState { GOAL_SEARCH, RETURN, FAST_PATH, NONE };
GoalState currentState = GoalState::GOAL_SEARCH;

MouseState mouseState{};

Solver* solver = nullptr;
const Goals* goal = &CENTER_GOALS;

FloodFillSolver floodFill{};
FastPathSolver fastPath{};
Solver noop = Solver{};
IO* io = nullptr;

MMSIO s = MMSIO{};
void switchState(GoalState state) {
  if (currentState == state) {
    return;
  }
  solver->onFinished(mouseState, goal);
  switch (state) {
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
  currentState = state;
  solver->init(mouseState, goal);
}
void updateState() {
  switch (currentState) {
    case GoalState::GOAL_SEARCH:
      if (solver->end(mouseState, goal)) switchState(GoalState::RETURN);
      break;
    case GoalState::RETURN:
      if (solver->end(mouseState, goal)) {
        CellSelection::search_all(mouseState);
        switchState(GoalState::FAST_PATH);
      }
      break;
    case GoalState::FAST_PATH:
      if (solver->end(mouseState, goal)) switchState(GoalState::RETURN);
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
  }
  io = &s;
  solver = &floodFill;
  goal = &CENTER_GOALS;
  currentState = GoalState::GOAL_SEARCH;
  io->init();
}
}  // namespace

int main() {
  init();
  while (true) {
    io->update(mouseState);
    updateState();
    Action* a = solver->run(mouseState, goal);
    a->run(mouseState, *io);
  }
}
