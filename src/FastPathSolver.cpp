#include "FastPathSolver.h"
CommandAction g_cmd{};

void FastPathSolver::init(MouseState &state, const Goals *goal) {
  // Serial.print("INIT");
  CellSelection::search_all(state, goal);
  // Serial.println(CellSelection::getCmds().size());
  g_cmd.load(std::move(CellSelection::getCmds()));
}

Action *FastPathSolver::run(MouseState &, const Goals *) { return &g_cmd; }
bool FastPathSolver::end(MouseState &, const Goals *) {
  if (g_cmd.completed()) {
    // Serial.print("END");
  }
  return g_cmd.completed();
}
void FastPathSolver::onFinished(MouseState &, const Goals *) { g_cmd.cancel(); }
