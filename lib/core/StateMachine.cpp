#include "StateMachine.h"

#include "robot/Robot.h"

Move relativeMove(Dir facing, Dir target) {
  switch ((static_cast<int>(target) - static_cast<int>(facing)) & 3) {
  case 0:
    return Move::Forward;
  case 1:
    return Move::Right;
  case 2:
    return Move::Back;
  default:
    return Move::Left;
  }
}

std::vector<Move> planFastMoves(const MazeMap &map, const Phase &phase) {
  const DistanceField field = floodFill(map, *phase.goal, phase.reach());
  std::vector<Move> moves;
  Cell at{0, 0};
  Dir facing = Dir::North;
  while (!atGoal(at, *phase.goal) && field.at(at) < INF &&
         moves.size() < N * N) {
    const Dir d = nextStep(map, field, at, facing, *phase.goal);
    moves.push_back(relativeMove(facing, d));
    at = step(at, d);
    facing = d;
  }
  moves.push_back(Move::Stop);
  return moves;
}

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
  // if (currentState == state) {
  //   return;
  // }
  // if (a && !a->completed()) {
  //   a->cancel();
  // }
  // r.map.allowUpdates(false);
  // phase = phaseFor(state);

  // switch (state) {
  // case GoalState::FAST_PATH:
  //   cmd.goalAngle = 0;
  //   startup = makeStartup();
  //   a = &startup;
  //   a->begin(r);
  //   r.driveDuty(0, 0);
  //   break;
  // case GoalState::NONE:
  //   a = &empty;
  //   a->begin(r);
  //   break;
  // case GoalState::GOAL_SEARCH:
  // case GoalState::RETURN:
  //   break;
  // }
  // currentState = state;
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

  // updateState(at, r);

  // if (a->completed()) {
  //   a->end(r);
  //   r.map.allowUpdates(phase.mapping);
  //   r.observe();
  //   cmd.load({exploreStep(r.map, at, facing, *phase.goal, phase.reach(),
  //                         phase.fastSpeed)});
  //   a = &cmd;
  //   a->begin(r);
  // }
  a->run(r);
}

void StateMachine::runFastPath(Robot &r, const std::vector<Move> &moves) {
  const std::vector<Command> commands = CommandGenerator{}.exec(moves);

  translator.goalAngle = 0;
  std::vector<std::unique_ptr<Action>> steps;
  for (const Command &c : commands)
    steps.push_back(std::make_unique<CommandStep>(c, translator, FAST_SPEED));

  fastPath = SequentialAction(std::move(steps));
  a = &fastPath;
  a->begin(r);
}
