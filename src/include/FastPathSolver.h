#pragma once
#include "Solver.h"
class FastPathSolver final : public Solver {
 public:
  Action* run(MouseState& state, const Goals* goal) override;
  bool end(MouseState& state, const Goals* goal) override;
  void logType() const override { log("fastPath"); }
  void onFinished(MouseState& state, const Goals* goal) override;
  void init(MouseState& state, const Goals* goal) override;
};
