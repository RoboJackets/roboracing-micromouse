#include "StateMachine.h"

namespace StateMachine {
GoalState currentState = GoalState::GOAL_SEARCH;

MouseState mouseState{};

Solver *solver = nullptr;
const Goals *goal = &CENTER_GOALS;

FloodFillSolver floodFill{};
FastPathSolver fastPath{};
Solver noop = Solver{};
EmptyAction empty = EmptyAction{};
SequentialAction square = SequentialAction::make(
    DelayAction(6), YawPIDAction(0), ProfiledDriveAction(0.3048, 0, 0.1),
    ProfiledCurveAction(0.05, -PI / 2, 0.1),
    ProfiledDriveAction(0.3048, -PI / 2, 0.4),
    ProfiledCurveAction(0.05, -PI / 2, 0.1),
    ProfiledDriveAction(0.3048, -PI, 0.4),
    ProfiledCurveAction(0.05, -PI / 2, 0.1),
    ProfiledDriveAction(0.3048, -1.5 * PI, 0), YawPIDAction(0));
DriveTimeAction vroom = DriveTimeAction(1000, 0.1);
ProfiledCurveAction pid = ProfiledCurveAction(1, 2 * PI, 0);
SysIDRampAction ramp{};

SequentialAction s =
    SequentialAction::make(DelayAction(6), ProfiledDriveAction(0.3048, 0, 0));
SequentialAction r =
    SequentialAction::make(DelayAction(6), YawPIDAction(PI / 2));
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
  if (a && !a->completed()) {
    a->cancel();
  }
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
    // a = solver->run(mouseState, goal); // determine action
    a = &empty;
  }
  a->run(mouseState, *io); // run action
}
} // namespace StateMachine