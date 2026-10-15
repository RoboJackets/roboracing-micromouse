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
  a->begin(r);
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
    startup = makeStartup();
    a = &startup;
    a->begin(r);
    r.driveDuty(0, 0);
    break;
  case GoalState::NONE:
    a = &empty;
    a->begin(r);
    break;
  case GoalState::GOAL_SEARCH:
    break;
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

  
  if (a->completed()) {
    updateState(at, r);
    a->end(r);
    r.map.allowUpdates(phase.mapping);
    r.observe();
    if (currentState == GoalState::FAST_PATH) {
      translator.goalAngle = 0;
      act = movesToAction(r, planFastRoute(r.map, *phase.goal), FAST_SPEED);
    } else {
      Move move = exploreStep(r.map, at, facing, *phase.goal, phase.reach(),
                              phase.fastSpeed);
      std::vector<Move> moves = {move, Move::Continue};
      act = movesToAction(r, moves, EXPLORE_SPEED);
    }

    a = &act;
    a->begin(r);
  }
  a->run(r);
}

SequentialAction StateMachine::movesToAction(Robot& r, const std::vector<Move> &moves,
                                             const SpeedProfile speed) {
  const std::vector<Command> commands = CommandGenerator{}.exec(moves);
  CommandTranslator translator{};
  std::vector<std::unique_ptr<Action>> steps;
  for (const Command &c : commands)
    steps.push_back(translator.translate(c, r, speed));
  return SequentialAction(std::move(steps));
}

void StateMachine::runFastPath(Robot &r, const std::vector<Move> &moves) {
  act = movesToAction(r, moves, FAST_SPEED);
  a = &act;
  a->begin(r);
}
