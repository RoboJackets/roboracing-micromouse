#include "StateMachine.h"

namespace StateMachine {
GoalState currentState = GoalState::GOAL_SEARCH;

MouseState mouseState{};

Solver *solver = nullptr;
const Goals *goal = &CENTER_GOALS;

FloodFillSolver floodFill{};
FloodFillSolver fastFloodFill{};
Solver noop = Solver{};
EmptyAction empty = EmptyAction{};
bool enableUpdatesAfterStartup = true;
SequentialAction startup =
    SequentialAction::make(DelayAction(3), StartupAction{});
Action *a = &startup;
void switchState(GoalState state, MouseIO *io) {
  if (currentState == state) {
    return;
  }
  if (a && !a->completed()) {
    a->cancel();
  }
  solver->onFinished(mouseState, goal);
  io->allowUpdates(false);
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
    solver = &fastFloodFill;
    goal = &CENTER_GOALS;
    enableUpdatesAfterStartup = false;
    a = &startup;
    io->driveVoltage(0, 0);
    break;
  default:
    solver = &noop;
    break;
  }
  currentState = state;
  solver->init(mouseState, goal);
}
void updateState(MouseIO *io) {
  switch (currentState) {
  case GoalState::GOAL_SEARCH:
    if (solver->end(mouseState, goal))
      switchState(GoalState::RETURN, io);
    break;
  case GoalState::RETURN:
    if (solver->end(mouseState, goal)) {
      switchState(GoalState::FAST_PATH, io);
    }
    break;
  case GoalState::FAST_PATH:
    if (solver->end(mouseState, goal))
      switchState(GoalState::RETURN, io);
    break;
  default:
    break;
  }
}
void init(MouseIO *io) {
  fastFloodFill.fast = true;
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
  updateState(io);        // determine overall goal (solver)
  if (a->completed()) {
    a->end(mouseState, *io);
    if (enableUpdatesAfterStartup) {
      io->allowUpdates(true);
    }
    a = solver->run(mouseState, goal); // determine action
  }
  a->run(mouseState, *io); // run action
}
} // namespace StateMachine