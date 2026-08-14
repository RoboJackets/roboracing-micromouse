#include "StateMachine.h"

void StateMachine::init(Robot &r) {
  r.map.markKnown(Cell{0, 0});
  for (int i = 0; i < CENTER_GOALS.count; ++i)
    r.map.markKnown(CENTER_GOALS.cells[i]);
  r.map.addBorderWalls();

  fast = false;
  goal = &CENTER_GOALS;
  currentState = GoalState::GOAL_SEARCH;
  r.init();
}

void StateMachine::switchState(GoalState state, Robot &r) {
  if (currentState == state) {
    return;
  }
  if (a && !a->completed()) {
    a->cancel();
  }
  r.map.allowUpdates(false);
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
    r.driveVoltage(0, 0);
    break;
  case GoalState::NONE:
    a = &empty;
    break;
  }
  currentState = state;
}

void StateMachine::updateState(Cell at, Robot &r) {
  switch (currentState) {
  case GoalState::GOAL_SEARCH:
    if (atGoal(at, *goal))
      switchState(GoalState::RETURN, r);
    break;
  case GoalState::RETURN:
    if (atGoal(at, *goal))
      switchState(GoalState::FAST_PATH, r);
    break;
  case GoalState::FAST_PATH:
    if (atGoal(at, *goal))
      switchState(GoalState::RETURN, r);
    break;
  case GoalState::NONE:
    break;
  }
}

void StateMachine::tick(Robot &r) {
  r.update();

  const WorldCoord pose = r.getWorldCoord();
  const Cell at = cellOf(pose);
  const Dir facing = dirOf(pose.theta);

  updateState(at, r);

  if (a->completed()) {
    a->end(r);
    if (enableUpdatesAfterStartup) {
      r.map.allowUpdates(true);
    }
    cmd.load({exploreStep(r.map, at, facing, *goal, fast)});
    a = &cmd;
  }
  a->run(r);
}
