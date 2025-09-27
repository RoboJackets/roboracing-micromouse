#include "include\FastPathSolver.h"

#include "include\CellSelection.h"
bool ran = false;
void FastPathSolver::run(MouseState& state, const Goals* goal) {
  if (!ran) {
    log(std::to_string(CellSelection::selectAllPaths(state, goal).size()));
    ran = true;
  }
}

bool FastPathSolver::end(MouseState& state, const Goals* goal) {
  return atGoal(state, goal);
}
