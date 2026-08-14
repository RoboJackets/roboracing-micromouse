#pragma once
#include "robot/Robot.h"

struct Action {
  virtual ~Action() = default;
  virtual void run(Robot &r) {};
  virtual void cancel() {};
  virtual bool completed() const { return true; };
  virtual void end(Robot &r) {};
};
