#pragma once
#include "Mouse.h"

class Solver {
 public:
  virtual ~Solver() = default;
  virtual void run(MouseState& state, const Goals* goal) {};
  virtual bool end(MouseState& state, const Goals* goal) { return true; };
  virtual void logType() const { log("none"); }
  virtual void init(MouseState& state, const Goals* goal) {}
  virtual void onFinished(MouseState& state, const Goals* goal) {}
};
