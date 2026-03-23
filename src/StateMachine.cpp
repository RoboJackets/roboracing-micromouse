#include "StateMachine.h"

namespace StateMachine {
GoalState currentState = GoalState::GOAL_SEARCH;

MouseState mouseState{};

Solver *solver = nullptr;
const Goals *goal = &TEST_GOALS;

FloodFillSolver floodFill{};
// FastPathSolver fastPath{};
FloodFillSolver fastFloodFill{};
Solver noop = Solver{};
EmptyAction empty = EmptyAction{};
bool enableUpdatesAfterStartup = true;
SequentialAction startup =
    SequentialAction::make(DelayAction(6), StartupAction{});
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

SequentialAction s = SequentialAction::make(
    DelayAction(6), ProfiledCurveAction(0.01, PI / 2, 0));
SequentialAction r =
    SequentialAction::make(DelayAction(3), YawPIDAction(PI / 2));
Action *a = &startup;
void switchState(GoalState state, MouseIO *io) {
  // Serial.print("1");
  if (currentState == state) {
    // Serial.print("2");
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
    goal = &TEST_GOALS;
    enableUpdatesAfterStartup = true;
    a = &startup;
    break;
  case GoalState::RETURN:
    solver = &floodFill;
    goal = &START_GOALS;
    break;
  case GoalState::FAST_PATH:
    // Serial.print("3");
    solver = &fastFloodFill;
    goal = &TEST_GOALS;
    enableUpdatesAfterStartup = false;
    a = &startup;
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
    // Serial.print("R");
    // std::cerr <<
    // std::to_string(mouseState.explored[mouseState.y][mouseState.x])
    //           << std::endl;
    if (solver->end(mouseState, goal)) {
      // Serial.print("0");
      switchState(GoalState::FAST_PATH, io);
    }
    break;
  case GoalState::FAST_PATH:
    // Serial.print("F");
    if (solver->end(mouseState, goal))
      switchState(GoalState::RETURN, io);
    break;
  default:
    break;
  }
  // Serial.println("!");
}
void init(MouseIO *io) {
  fastFloodFill.fast = true;
  mouseState.explored[0][0] = true;
  for (int i = 0; i < TEST_GOALS.count; ++i) {
    const int gx = TEST_GOALS.cells[i][1];
    const int gy = TEST_GOALS.cells[i][0];
    mouseState.explored[gy][gx] = true;
  }
  for (int i = 0; i < N; ++i) {
    mouseState.walls[i][0] |= LEFT;
    mouseState.walls[0][i] |= DOWN;
    mouseState.walls[i][N - 1] |= RIGHT;
    mouseState.walls[N - 1][i] |= TOP;
  }
  solver = &floodFill;
  goal = &TEST_GOALS;
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
    // a = &empty;
  }
  a->run(mouseState, *io); // run action
}
} // namespace StateMachine