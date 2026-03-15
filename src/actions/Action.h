#pragma once
#include "IdealState.h"
#include "Mouse.h"
#include "MouseIO.h"
#include "Types.h"
struct Action
{
  virtual ~Action() = default;
  virtual void run(MouseState &state, MouseIO &io) {};
  virtual IdealState getIdealState() { return IdealState{}; }
  virtual void cancel() {};
  virtual bool completed() const { return true; };
  virtual void end(MouseState &s, MouseIO &io) {};
};