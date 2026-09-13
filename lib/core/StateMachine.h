#pragma once
#include "actions/CommandAction.h"
#include "actions/EmptyAction.h"
#include "actions/SequentialAction.h"
#include "actions/StartupAction.h"
#include "maze/Explorer.h"

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

  SequentialAction startup = makeStartup();
  CommandAction cmd{};
  EmptyAction empty{};
  Action *a = &startup;

  static SequentialAction makeStartup() {
    return SequentialAction::make(DelayAction(3), StartupAction{});
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
  void switchState(GoalState state, Robot &r);
  void updateState(Cell at, Robot &r);
};
