#pragma once
#include "robot/Robot.h"
#include "maze/MazeMap.h"

struct Action {
  virtual ~Action() = default;
  virtual void run(MazeMap &map, Robot &r) {};
  virtual void cancel() {};
  virtual bool completed() const { return true; };
  virtual void end(MazeMap &map, Robot &r) {};
};
