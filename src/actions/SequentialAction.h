#pragma once
#include "Action.h"
#include "CommandGenerator.h"
#include "Commands.h"
#include <memory>
#include <vector>
struct SequentialAction : Action {
  bool canceled = false;
  std::vector<std::unique_ptr<Action>> actions{};
  size_t index = 0;
  SequentialAction() = default;
  SequentialAction(std::vector<std::unique_ptr<Action>> actions)
      : actions(std::move(actions)) {}

  template <typename... Args> static SequentialAction make(Args &&...args) {
    std::vector<std::unique_ptr<Action>> v;
    (v.push_back(
         std::make_unique<std::decay_t<Args>>(std::forward<Args>(args))),
     ...);
    return SequentialAction(std::move(v));
  }
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled; }

  void run(MouseState &s, MouseIO &io) override {
    if (index >= actions.size()) {
      return;
    }
    if (actions[index]->completed()) {
      actions[index]->end(s, io);
      index++;
    }
    if (index == actions.size()) {
      canceled = true;
      return;
    }
    actions[index]->run(s, io);
  }
  void end(MouseState &s, MouseIO &io) override {}
};
