#pragma once
#include "CommandAction.h"
#include "CommandGenerator.h"
#include "MoveAction.h"
#include "Solver.h"
#include <queue>
#include <string>


class FloodFillSolver final : public Solver {
public:
  CommandAction cmd{};
  Action *run(MouseState &state, const Goals *goal) override;
  bool end(MouseState &state, const Goals *goal) override;
  void logType() const override { log("floodFill"); }
};
