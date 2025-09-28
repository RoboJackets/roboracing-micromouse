#pragma once
#include "Types.h"
#include "IdealState.h"
#include "Mouse.h"
#include "IO.h"
struct Action {
  virtual void init() {};
  virtual void run(MouseState& state, IO& io) {};
  virtual void onEnd() {};
  virtual bool isFinished() { return true; };
  virtual IdealState getIdealState() { return IdealState{}; }
};