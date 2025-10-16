#pragma once
#include "Action.h"
struct EmptyAction : Action {
  bool completed() const override { return true; }
};