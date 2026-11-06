#pragma once
#include "actions/Action.h"
#include "actions/EmptyAction.h"
#include "actions/StartupAction.h"
#include "maze/Explorer.h"
#include "planner/Commands.h"
#include "planner/CellSearch.h"

enum class GoalState { GOAL_SEARCH, RETURN, FAST_PATH, NONE };

struct Phase {
  const Goals *goal;
  bool fastSpeed;
  bool mapping;

  Reachability reach() const {
    return mapping ? Reachability::AllCells : Reachability::KnownCellsOnly;
  }
};

struct StateMachine {
  GoalState currentState = GoalState::GOAL_SEARCH;
  Phase phase = phaseFor(GoalState::GOAL_SEARCH);

  CommandTranslator translator{};
  std::unique_ptr<Action> a = makeStartup();
  bool startingUp = true; // a is the startup sequence; reset the translator when it ends

  static std::unique_ptr<Action> makeStartup() {
    return std::make_unique<SequentialAction>(
        SequentialAction::make(DelayAction(3), StartupAction{}));
  }

  static constexpr Phase phaseFor(GoalState state) {
    switch (state) {
    case GoalState::GOAL_SEARCH:
      return {&CENTER_GOALS, false, true};
    case GoalState::RETURN:
      return {&START_GOALS, false, true};
    case GoalState::FAST_PATH:
      return {&CENTER_GOALS, true, false};
    case GoalState::NONE:
      return {&CENTER_GOALS, false, false};
    }
    return {&CENTER_GOALS, false, true};
  }

  void init(Robot &r);
  void tick(Robot &r);
  std::unique_ptr<Action> explore(Robot &r);
  void runFastPath(Robot &r, const std::vector<Move> &moves);
  // both return whether they replaced the current action
  bool switchState(GoalState state, Robot &r);
  bool updateState(Cell at, Robot &r);
  std::unique_ptr<Action> movesToAction(Robot& r, const std::vector<Move> &moves, const SpeedProfile speed);
};
