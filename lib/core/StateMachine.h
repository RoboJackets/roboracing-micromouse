#pragma once
#include "actions/ControlActions.h"
#include "actions/EmptyAction.h"
#include "actions/SequentialAction.h"
#include "actions/StartupAction.h"
#include "maze/FloodFillSolver.h"


enum class GoalState { GOAL_SEARCH, RETURN, FAST_PATH, NONE };
namespace StateMachine {
void tick(MouseIO *io);
void switchState(GoalState state, MouseIO *io);
void updateState(MouseIO *io);
void init(MouseIO *io);
} // namespace StateMachine