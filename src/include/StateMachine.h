#pragma once
#include "CellSelection.h"
#include "EmptyAction.h"
#include "FastPathSolver.h"
#include "FloodFillSolver.h"
#include "MMSIO.h"
enum class GoalState { GOAL_SEARCH, RETURN, FAST_PATH, NONE };
namespace StateMachine {
void tick(MouseIO* io);
void switchState(GoalState state);
void updateState();
void init(MouseIO* io);
}  // namespace