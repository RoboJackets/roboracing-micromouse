#pragma once
#include "IdealState.h"
#include "Mouse.h"
#include "MouseIO.h"
#include "Types.h"
struct Action {
  virtual void run(MouseState& state, MouseIO& io) {};
  virtual IdealState getIdealState() { return IdealState{}; }
};