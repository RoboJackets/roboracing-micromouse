#include "../include/FastPathSolver.h"

#include "../Actions/MoveAction.h"
#include "../include/CellSelection.h"
bool ran = false;
Path path = Path{};
MoveAction action = MoveAction();
int i = 1;
Action* FastPathSolver::run(MouseState& state, const Goals* goal) {
  if (i < path.steps.size()) {
    action.setIdealState(IdealState{path.steps[i]});
    // std::cerr << "x: " << std::to_string(path.steps[i].x)
    //           << "y: " << std::to_string(path.steps[i].y) << std::endl;
    i++;
  }
  return &action;
}

bool FastPathSolver::end(MouseState& state, const Goals* goal) {
  return atGoal(state, goal);
}

void FastPathSolver::init(MouseState& state, const Goals* goal) {
  i = 1;
  path = CellSelection::selectPath(state, goal);
}

void FastPathSolver::onFinished(MouseState& state, const Goals* goal) {}
