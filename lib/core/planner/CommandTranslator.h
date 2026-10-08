#pragma once
#include <memory>
#include <vector>

#include "actions/Action.h"
#include "CommandGenerator.h"
#include "Commands.h"
#include "Tuning.h"

struct CommandTranslator {
  int goalAngle = 0;

  std::unique_ptr<Action> translate(const Command& c, Robot& r, const SpeedProfile& sp);

private:
  std::unique_ptr<Action> makeFwdAction(int8_t amt, Robot &r,
                                        const SpeedProfile &sp);

  std::unique_ptr<Action> makeCurveAction(int8_t amt, Robot &r,
                                          const SpeedProfile &sp);
};