#include "StateMachine.h"

#include "robot/Robot.h"

void StateMachine::init(Robot &r) {
  r.map.markKnown(Cell{0, 0});
  for (int i = 0; i < CENTER_GOALS.count; ++i)
    r.map.markKnown(CENTER_GOALS.cells[i]);
  r.map.addBorderWalls();

  currentState = GoalState::GOAL_SEARCH;
  phase = phaseFor(currentState);
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
  phase = phaseFor(state);

  switch (state) {
  case GoalState::FAST_PATH:
    cmd.goalAngle = 0;
    startup = makeStartup();
    a = &startup;
    r.driveDuty(0, 0);
    break;
  case GoalState::NONE:
    a = &empty;
    break;
  case GoalState::GOAL_SEARCH:
  case GoalState::RETURN:
    break;
  }
  currentState = state;
}

void StateMachine::updateState(Cell at, Robot &r) {
  switch (currentState) {
  case GoalState::GOAL_SEARCH:
    if (atGoal(at, *phase.goal))
      switchState(GoalState::RETURN, r);
    break;
  case GoalState::RETURN:
    if (atGoal(at, *phase.goal))
      switchState(GoalState::FAST_PATH, r);
    break;
  case GoalState::FAST_PATH:
    if (atGoal(at, *phase.goal))
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
    r.map.allowUpdates(phase.mapping);
    cmd.load({exploreStep(r.map, at, facing, *phase.goal, phase.reach(),
                          phase.fastSpeed)});
    a = &cmd;
  }
  a->run(r);
}
