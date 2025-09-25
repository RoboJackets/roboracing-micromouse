#include <queue>
#include <string>

#include "FloodFillSolver.cpp"
MouseState mouseState{};
enum State { GOAL_SEARCH, RETURN, FAST_PATH, NONE };
State currentState = GOAL_SEARCH;
Solver* solver{};
const Goals* goal = &CENTER_GOALS;

void updateState() {
  switch (currentState) {
    case GOAL_SEARCH:
      if (solver->end(mouseState, goal)) {
        currentState = RETURN;
      }
      break;
    case RETURN:
      if (solver->end(mouseState, goal)) {
        currentState = NONE;
      }
      break;
    default:
      break;
  }
}
void init() {
  mouseState.explored[0][0] = true;
  for (int i = 0; i < CENTER_GOALS.count; ++i) {
    const int gx = CENTER_GOALS.cells[i][1];
    const int gy = CENTER_GOALS.cells[i][0];
    mouseState.explored[gy][gx] = true;
  }

  for (int i = 0; i < N; ++i) {
    mouseState.walls[i][0] |= LEFT;
    mouseState.walls[0][i] |= DOWN;
    mouseState.walls[i][N - 1] |= RIGHT;
    mouseState.walls[N - 1][i] |= TOP;

    API::setWall(0, i, 'w');
    API::setWall(i, 0, 's');
    API::setWall(N - 1, i, 'e');
    API::setWall(i, N - 1, 'n');
  }

  log("Running...");
  API::setColor(0, 0, 'G');
  API::setText(0, 0, "start");
}
int main(int argc, char const* argv[]) {
  init();
  FloodFillSolver floodFillSolver{};
  Solver noop{};
  while (true) {
    switch (currentState) {
      case GOAL_SEARCH:
        solver = &floodFillSolver;
        goal = &CENTER_GOALS;
        break;
      case RETURN:
        goal = &START_GOALS;
        break;
      default:
        solver = &noop;
        break;
    }
    solver->run(mouseState, goal);
    updateState();
  }
}