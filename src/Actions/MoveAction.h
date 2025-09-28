#pragma once
#include "../include/Action.h"
struct MoveAction : Action {
  IdealState state;
  MoveAction(IdealState s) : state(s) {}
  IdealState getIdealState() override { return state; }
  void run(MouseState& state, IO& io) override {
    state.explored[state.y][state.x] = true;
    io.setState(getIdealState());
  }
};