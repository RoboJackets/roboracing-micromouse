#pragma once
#include "Types.h"
#include "IdealState.h"
#include "Mouse.h"
#include "IO.h"
struct Action {
  virtual void run(MouseState& state, IO& io) {};
  virtual IdealState getIdealState() { return IdealState{}; }
};