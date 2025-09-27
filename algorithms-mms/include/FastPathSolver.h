#pragma once
#include "Solver.h"
class FastPathSolver final : public Solver {
 public:
  void run(MouseState& state, const Goals* goal) override;
  bool end(MouseState& state, const Goals* goal) override;
  void logType() const override { log("fastPath"); }
};
