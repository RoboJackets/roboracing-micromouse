#pragma once
#include "CellSelection.h"
#include "ControlActions.h"
#include "EmptyAction.h"
#include "FastPathSolver.h"
#include "FloodFillSolver.h"
#include "SequentialAction.h"
#include "StartupAction.h"


enum class GoalState { GOAL_SEARCH, RETURN, FAST_PATH, NONE };
namespace StateMachine {
void tick(MouseIO *io);
void switchState(GoalState state, MouseIO *io);
void updateState(MouseIO *io);
void init(MouseIO *io);
} // namespace StateMachine