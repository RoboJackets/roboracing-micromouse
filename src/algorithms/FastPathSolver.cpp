#include "FastPathSolver.h"

#include "CellSelection.h"
#include "CommandAction.h"
#include "CommandGenerator.h"
CommandAction g_cmd{};

void FastPathSolver::init(MouseState& state, const Goals* goal) {
  CellSelection::search_all(state);
  g_cmd.load(std::move(CellSelection::getCmds()));
}

Action* FastPathSolver::run(MouseState&, const Goals*) { return &g_cmd; }
bool FastPathSolver::end(MouseState&, const Goals*) {
  return g_cmd.completed();
}
void FastPathSolver::onFinished(MouseState&, const Goals*) { g_cmd.cancel(); }
