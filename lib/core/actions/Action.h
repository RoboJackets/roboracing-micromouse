#pragma once
#include "MouseIO.h"
#include "maze/MazeMap.h"

struct Action {
  virtual ~Action() = default;
  virtual void run(MazeMap &map, MouseIO &io) {};
  virtual void cancel() {};
  virtual bool completed() const { return true; };
  virtual void end(MazeMap &map, MouseIO &io) {};
};
