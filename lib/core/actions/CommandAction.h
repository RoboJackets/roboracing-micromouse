#pragma once
#include <memory>
#include <vector>

#include "Action.h"
#include "Commands.h"
#include "ControlActions.h"
#include "EmptyAction.h"
#include "SequentialAction.h"
#include "StartupAction.h"
#include "Tuning.h"

struct CommandAction : Action {
  std::vector<unsigned char> buf;
  size_t pc = 0;
  std::unique_ptr<Action> curr;
  int goalAngle = 0;

  void load(std::vector<unsigned char> b);
  bool completed() const override {
    return canceled || (pc >= buf.size() && !curr);
  }
  void cancel() override;

  void run(Robot &r) override;
  void end(Robot &r) override;

  // arg encodes direction and magnitude in lower 5 bits
  // bit 4 = right(1)/left(0), bits 0-2 = 45*n degrees
  static int turnAmount(unsigned char arg);

  std::unique_ptr<Action> makeFwdAction(unsigned char arg, Robot &r,
                                        const SpeedProfile &sp);

  std::unique_ptr<Action> makeCurveAction(unsigned char arg, Robot &r,
                                          const SpeedProfile &sp);

  std::unique_ptr<Action> determineAction(Robot &r);
};
