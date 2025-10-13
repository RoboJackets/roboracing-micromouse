#pragma once
#include "Action.h"
struct MoveAction : Action {
  IdealState state;
  MoveAction() {}
  IdealState getIdealState() override { return state; }
  void setIdealState(IdealState newState) { state = newState; }
  void run(MouseState& state, MouseIO& io) override {
    state.explored[state.y][state.x] = true;
    io.setState(getIdealState());
  }
};