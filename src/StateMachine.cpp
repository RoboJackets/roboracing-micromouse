#include "StateMachine.h"
#include "ControlActions.h"
#include "SequentialAction.h"
#include <string>

namespace StateMachine {
GoalState currentState = GoalState::GOAL_SEARCH;

MouseState mouseState{};

Solver *solver = nullptr;
const Goals *goal = &CENTER_GOALS;

FloodFillSolver floodFill{};
FastPathSolver fastPath{};
Solver noop = Solver{};
EmptyAction empty = EmptyAction{};
SequentialAction square = SequentialAction(
    {DriveTimeAction(1, 0.5), YawPIDAction(M_PI / 2), DriveTimeAction(1, 0.5),
     YawPIDAction(M_PI), DriveTimeAction(1, 0.5), YawPIDAction(M_PI * 1.5),
     DriveTimeAction(1, 0.5), YawPIDAction(0)});
DriveTimeAction vroom = DriveTimeAction(1, 0.1);
Action *a = &empty;
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
  if (a && !a->completed())
    a->cancel();
  a = &empty;
  solver->init(mouseState, goal);
}
void updateState() {
  switch (currentState) {
  case GoalState::GOAL_SEARCH:
    if (solver->end(mouseState, goal))
      switchState(GoalState::RETURN);
    break;
  case GoalState::RETURN:
    // std::cerr <<
    // std::to_string(mouseState.explored[mouseState.y][mouseState.x])
    //           << std::endl;
    if (solver->end(mouseState, goal)) {
      switchState(GoalState::FAST_PATH);
    }
    break;
  case GoalState::FAST_PATH:
    if (solver->end(mouseState, goal))
      switchState(GoalState::RETURN);
    break;
  default:
    break;
  }
}
void init(MouseIO *io) {
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
  solver = &floodFill;
  goal = &CENTER_GOALS;
  currentState = GoalState::GOAL_SEARCH;
  io->init();
}
void tick(MouseIO *io) {
  io->update(mouseState); // update input states
  updateState();          // determine overall goal (solver)
  if (a->completed()) {
    a->end(mouseState, *io);
    a = &vroom; // determine action
  }
  a->run(mouseState, *io); // run action
}
} // namespace StateMachine