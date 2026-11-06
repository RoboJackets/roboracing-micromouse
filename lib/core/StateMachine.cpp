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

bool StateMachine::switchState(GoalState state, Robot &r) {
  if (currentState == state) {
    return false;
  }
  if (a && !a->completed()) {
    a->cancel();
  }
  r.map.allowUpdates(false);
  phase = phaseFor(state);

  bool replaced = false;
  switch (state) {
  case GoalState::FAST_PATH:
    a = makeStartup();
    startingUp = true;
    r.controller.enabled = false; // robot gets picked up and placed; re-anchored after startup
    a->begin(r);
    r.driveDuty(0, 0);
    replaced = true;
    break;
  case GoalState::NONE:
    a = std::make_unique<EmptyAction>();
    a->begin(r);
    replaced = true;
    break;
  case GoalState::GOAL_SEARCH:
    break;
  case GoalState::RETURN:
    break;
  }
  currentState = state;
  return replaced;
}

bool StateMachine::updateState(Cell at, Robot &r) {
  switch (currentState) {
  case GoalState::GOAL_SEARCH:
    if (atGoal(at, *phase.goal))
      return switchState(GoalState::RETURN, r);
    break;
  case GoalState::RETURN:
    if (atGoal(at, *phase.goal))
      return switchState(GoalState::FAST_PATH, r);
    break;
  case GoalState::FAST_PATH:
    if (atGoal(at, *phase.goal))
      return switchState(GoalState::RETURN, r);
    break;
  case GoalState::NONE:
    break;
  }
  return false;
}

void StateMachine::tick(Robot &r) {
  r.update();

  if (a->completed()) {
    a->end(r);
    // startup just placed the robot in the start cell facing north
    if (startingUp) {
      translator.reset(Cell{0, 0}, r.getPose(), 0);
      r.startController();
      startingUp = false;
    }
    // odometry can't tell which cell the robot is in when a drive ends exactly
    // on a cell edge, so use where the plan says it is
    const Cell at = translator.getCell();
    const Dir facing = translator.getFacing();
    if (updateState(at, r)) {
      a->run(r);
      return;
    }
    r.map.allowUpdates(phase.mapping);
    r.observe(at);
    if (currentState == GoalState::FAST_PATH) {
      printf("[fast_path]\n");
      std::vector<Move> route = planFastRoute(r.map, *phase.goal, at, facing);
      r.io.printf("Fast Route:\n");
      for (Move m : route) {
        r.io.printf("%c", to_char(m));
      }
      r.io.printf("\n");
      a = movesToAction(r, route, FAST_SPEED);
    } else {
      a = explore(r);
    }
    a->begin(r);
  }
  a->run(r);
}

std::unique_ptr<Action> StateMachine::explore(Robot& r) {
  SpeedProfile profile = EXPLORE_SPEED;

  double v0 = 0.5 * (std::abs(r.getDriveSpeedLeft()) + std::abs(r.getDriveSpeedRight()));
  double vf = std::min(profile.maxSpeed, std::sqrt(2 * profile.maxDecel * CELL_SIZE_METERS * 0.5));

  const Cell at = translator.getCell();
  const Dir facing = translator.getFacing();

  Move move = exploreStep(r.map, at, facing, *phase.goal, phase.reach(), phase.fastSpeed);
  printf("[explore] move: %d\n", move);
  
  const std::vector<Move> moves = {move, Move::Continue};
  const std::vector<Command> commands = CommandGenerator(true).exec(moves);

  std::vector<std::unique_ptr<Action>> steps;
  for (const Command& c : commands) {
    steps.push_back(translator.translate(c, r, profile, vf, vf));
  }
  return std::make_unique<SequentialAction>(std::move(steps));
}

std::unique_ptr<Action> StateMachine::movesToAction(Robot& r, const std::vector<Move> &moves,
                                                    const SpeedProfile speed) {
  const std::vector<Command> commands = CommandGenerator{}.exec(moves);
  std::vector<std::unique_ptr<Action>> steps;
  for (const Command &c : commands)
    steps.push_back(
        std::make_unique<DeferredCommandAction>(translator, c, speed));
  return std::make_unique<SequentialAction>(std::move(steps));
}

void StateMachine::runFastPath(Robot &r, const std::vector<Move> &moves) {
  a = movesToAction(r, moves, FAST_SPEED);
  a->begin(r);
}
