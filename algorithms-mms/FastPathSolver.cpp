#include "include\FastPathSolver.h"

#include "include\CellSelection.h"
bool ran = false;
Path & path;
int steps = 0;
void FastPathSolver::run(MouseState& state, const Goals* goal) {
  if (steps < path.steps.size()) {
    
  }
}

bool FastPathSolver::end(MouseState& state, const Goals* goal) {
  return atGoal(state, goal);
}

void FastPathSolver::init(MouseState& state, const Goals* goal) {
  path = CellSelection::selectPath(state, goal);
}

void FastPathSolver::onFinished(MouseState& state, const Goals* goal) {}
