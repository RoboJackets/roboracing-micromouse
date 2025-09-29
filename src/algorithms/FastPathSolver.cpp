#include "../include/FastPathSolver.h"

#include "../include/CellSelection.h"
bool ran = false;
Path path = Path{};
int steps = 0;
Action a = Action{};
Action* FastPathSolver::run(MouseState& state, const Goals* goal) {
  if (steps < path.steps.size()) {
  }
  return &a;
}

bool FastPathSolver::end(MouseState& state, const Goals* goal) {
  return atGoal(state, goal);
}

void FastPathSolver::init(MouseState& state, const Goals* goal) {
  path = CellSelection::selectPath(state, goal);
}

void FastPathSolver::onFinished(MouseState& state, const Goals* goal) {}
