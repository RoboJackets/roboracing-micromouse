#pragma once
#include "Mouse.h"
#include "actions/Action.h"

class Solver {
 public:
  Action a = Action{};
  virtual ~Solver() = default;
  virtual Action* run(MouseState& state, const Goals* goal) { return &a; };
  virtual bool end(MouseState& state, const Goals* goal) { return true; };
  virtual void init(MouseState& state, const Goals* goal) {}
  virtual void onFinished(MouseState& state, const Goals* goal) {}
};
