#include "StateMachine.h"

void StateMachine::init(MouseIO &io) {
  map.markKnown(Cell{0, 0});
  for (int i = 0; i < CENTER_GOALS.count; ++i)
    map.markKnown(CENTER_GOALS.cells[i]);
  map.addBorderWalls();

  fast = false;
  goal = &CENTER_GOALS;
  currentState = GoalState::GOAL_SEARCH;
  io.init();
}

void StateMachine::switchState(GoalState state, MouseIO &io) {
  if (currentState == state) {
    return;
  }
  if (a && !a->completed()) {
    a->cancel();
  }
  map.allowUpdates(false);
  switch (state) {
  case GoalState::GOAL_SEARCH:
    fast = false;
    goal = &CENTER_GOALS;
    break;
  case GoalState::RETURN:
    fast = false;
    goal = &START_GOALS;
    break;
  case GoalState::FAST_PATH:
    fast = true;
    goal = &CENTER_GOALS;
    enableUpdatesAfterStartup = false;
    cmd.goalAngle = 0;
    startup = makeStartup();
    a = &startup;
    io.driveVoltage(0, 0);
    break;
  case GoalState::NONE:
    a = &empty;
    break;
  }
  currentState = state;
}

void StateMachine::updateState(Cell at, MouseIO &io) {
  switch (currentState) {
  case GoalState::GOAL_SEARCH:
    if (atGoal(at, *goal))
      switchState(GoalState::RETURN, io);
    break;
  case GoalState::RETURN:
    if (atGoal(at, *goal))
      switchState(GoalState::FAST_PATH, io);
    break;
  case GoalState::FAST_PATH:
    if (atGoal(at, *goal))
      switchState(GoalState::RETURN, io);
    break;
  case GoalState::NONE:
    break;
  }
}

void StateMachine::tick(MouseIO &io) {
  io.update();

  const WorldCoord pose = io.getWorldCoord();
  const Cell at = cellOf(pose);
  const Dir facing = dirOf(pose.theta);

  updateState(at, io);

  if (a->completed()) {
    a->end(map, io);
    if (enableUpdatesAfterStartup) {
      map.allowUpdates(true);
    }
    cmd.load({exploreStep(map, at, facing, *goal, fast)});
    a = &cmd;
  }
  a->run(map, io);
}
