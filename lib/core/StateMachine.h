#pragma once
#include "actions/CommandAction.h"
#include "actions/EmptyAction.h"
#include "actions/SequentialAction.h"
#include "actions/StartupAction.h"
#include "maze/Explorer.h"

enum class GoalState { GOAL_SEARCH, RETURN, FAST_PATH, NONE };

struct StateMachine {
  GoalState currentState = GoalState::GOAL_SEARCH;
  MazeMap map{};
  const Goals *goal = &CENTER_GOALS;
  bool fast = false;
  bool enableUpdatesAfterStartup = true;

  SequentialAction startup = makeStartup();
  CommandAction cmd{};
  EmptyAction empty{};
  Action *a = &startup;

  static SequentialAction makeStartup() {
    return SequentialAction::make(DelayAction(3), StartupAction{});
  }

  void init(Robot &r);
  void tick(Robot &r);
  void switchState(GoalState state, Robot &r);
  void updateState(Cell at, Robot &r);
};
