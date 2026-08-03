#pragma once
#include <memory>
#include <vector>

#include "Action.h"
#include "Commands.h"
#include "ControlActions.h"
#include "EmptyAction.h"
#include "SequentialAction.h"
#include "StartupAction.h"

struct SpeedProfile {
  double maxSpeed;
  double driveFinalVelocity;
  double curveRadius;
  double curveFinalVelocity;
  double curveTrailDistance;
};

inline constexpr SpeedProfile EXPLORE_SPEED{0.1, 0.1, 0.03, 0.1, 0.03};
inline constexpr SpeedProfile FAST_SPEED{0.2, 0.2, 0.03, 0.2, 0.04};

struct CommandAction : Action {
  std::vector<unsigned char> buf;
  size_t pc = 0;
  bool canceled = false;
  std::unique_ptr<Action> curr;
  GridCoord goal{};
  int goalAngle = 0;

  void load(std::vector<unsigned char> b);
  void cancel() override { canceled = true; }
  bool completed() const override {
    return canceled || (pc >= buf.size() && !curr);
  }

  void run(MouseState &s, MouseIO &io) override;

  // arg encodes direction and magnitude in lower 5 bits
  // bit 4 = right(1)/left(0), bits 0-2 = 45*n degrees
  static int turnAmount(unsigned char arg);

  std::unique_ptr<Action> makeFwdAction(unsigned char arg, MouseIO &io,
                                        MouseState &s, const SpeedProfile &sp);

  std::unique_ptr<Action> makeCurveAction(unsigned char arg, MouseIO &io,
                                          MouseState &s,
                                          const SpeedProfile &sp);

  std::unique_ptr<Action> determineAction(MouseState &s, MouseIO &io);
};
