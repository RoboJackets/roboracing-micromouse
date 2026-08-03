#pragma once
#include "Solver.h"
#include "actions/CommandAction.h"
#include <queue>
#include <string>

class FloodFillSolver final : public Solver {
public:
  CommandAction cmd{};
  bool fast = false;
  Action *run(MouseState &state, const Goals *goal) override;
  bool end(MouseState &state, const Goals *goal) override;
};
