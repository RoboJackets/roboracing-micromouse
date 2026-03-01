#include <SingleCommandSolver.h>
struct SingleCommandSolver : Solver {
  Action *a = nullptr;
  SingleCommandSolver(Action *a) : a(a) {}
  Action *run(MouseState &state, const Goals *goal) override { return a; };
  bool end(MouseState &state, const Goals *goal) override {
    return a->completed();
  };
  void logType() const override { log("single command"); }
};