#include "include\FastPathSolver.h"

void FastPathSolver::run(MouseState&, const Goals*) {}

bool FastPathSolver::end(const MouseState& state, const Goals* goal) {
  return atGoal(state, goal);
}
