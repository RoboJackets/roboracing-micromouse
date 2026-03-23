#pragma once
#include <memory>
#include <vector>

#include "Action.h"
#include "CommandGenerator.h"
#include "Commands.h"
#include "ControlActions.h"
#include "EmptyAction.h"
#include "MMSIO.h"
#include "SequentialAction.h"
#include <StartupAction.h>

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

  void load(std::vector<unsigned char> b) {
    buf = std::move(b);
    canceled = false;
    pc = 0;
    curr.reset();
  }
  void cancel() override { canceled = true; }
  bool completed() const override {
    return canceled || (pc >= buf.size() && !curr);
  }

  void run(MouseState &s, MouseIO &io) override {
    if (completed())
      return;
    if (io.isMMS()) {
      runMMS(s, io);
      return;
    }
    if (!curr) {
      curr = determineAction(s, io);
    }
    curr->run(s, io);
    if (curr->completed()) {
      curr->end(s, io);
      s.x = io.getGridCoord().x;
      s.y = io.getGridCoord().y;
      s.dir = io.getGridCoord().dir;
      curr.reset();
    }
  }

  static int turnAmount(unsigned char arg) {
    // arg encodes direction and magnitude in lower 5 bits
    // bit 4 = right(1)/left(0), bits 0-2 = 45*n degrees
    if (arg == 0)
      return 0;
    int magnitude = 0;
    unsigned char lower = arg & 0b00000111;
    if (lower == 1)
      magnitude = 1; // 45
    else if (lower == 2)
      magnitude = 2; // 90
    else if (lower == 4)
      magnitude = 3; // 135
    bool right = (arg & 0b00010000) != 0;
    return right ? magnitude : -magnitude;
  }

  std::unique_ptr<Action> makeFwdAction(unsigned char arg, MouseIO &io,
                                        MouseState &s, const SpeedProfile &sp) {
    GridCoord v = angleToVector(goalAngle);
    goal.x += v.x * arg;
    goal.y += v.y * arg;

    WorldCoord rel = io.getWorldCoord().gridRelativeCoords(io.getGridCoord());
    double halfCell = CELL_SIZE_METERS / 2.0;
    double dx =
        v.x != 0
            ? (v.x * arg * CELL_SIZE_METERS + halfCell - rel.x - v.x * 0.01)
            : 0;
    double dy =
        v.y != 0
            ? (v.y * arg * CELL_SIZE_METERS + halfCell - rel.y - v.y * 0.01)
            : 0;

    double distance = std::sqrt(dx * dx + dy * dy);
    double travelAngle = M_PI / 2.0 - goalAngle * M_PI / 4.0;
    // Serial.printf("FWD%d    WALL: %d\n", arg,
    //               s.walls[io.getGridCoord().x][io.getGridCoord().y]);
    return std::make_unique<SequentialAction>(
        SequentialAction::make(ProfiledDriveAction{
            distance, travelAngle, sp.driveFinalVelocity, sp.maxSpeed}));
  }

  std::unique_ptr<Action> makeCurveAction(unsigned char arg, MouseIO &io,
                                          MouseState &s,
                                          const SpeedProfile &sp) {
    // Serial.printf("CURVE    WALL: %d\n",
    //               s.walls[io.getGridCoord().x][io.getGridCoord().y]);
    goalAngle += turnAmount(arg);
    double targetTheta = M_PI / 2.0 - goalAngle * M_PI / 4.0;
    double currentTheta = io.getWorldCoord().theta;
    double turnAngle = std::atan2(std::sin(targetTheta - currentTheta),
                                  std::cos(targetTheta - currentTheta));

    goalAngle = (goalAngle + 8) % 8;

    GridCoord v = angleToVector(goalAngle);
    goal.x += v.x;
    goal.y += v.y;
    double travelAngle = M_PI / 2.0 - goalAngle * M_PI / 4.0;
    return std::make_unique<SequentialAction>(SequentialAction::make(
        ProfiledCurveAction(sp.curveRadius, turnAngle, sp.curveFinalVelocity,
                            sp.maxSpeed),
        ProfiledDriveAction{sp.curveTrailDistance, travelAngle,
                            sp.driveFinalVelocity, sp.maxSpeed}));
  }

  std::unique_ptr<Action> determineAction(MouseState &s, MouseIO &io) {
    unsigned char c = buf[pc++];

    unsigned char cls = c & 0b11100000;
    unsigned char arg = c & 0b00011111;
    if (c == STOP) {
      io.driveVoltage(0, 0);
      canceled = true;
      return std::make_unique<EmptyAction>();
    }
    if (c == IPT180) {
      goalAngle += 4;
      goalAngle = (goalAngle + 8) % 8;
      io.driveVoltage(0, 0);
      double theta = M_PI / 2.0 - goalAngle * M_PI / 4.0;
      double currentTheta = io.getWorldCoord().theta;
      double turnAngle = std::atan2(std::sin(theta - currentTheta),
                                    std::cos(theta - currentTheta));
      // Serial.printf("IPT%d    WALL: %d\n", arg,
      //               s.walls[io.getGridCoord().x][io.getGridCoord().y]);
      return std::make_unique<SequentialAction>(SequentialAction::make(
          ProfiledRotationAction{turnAngle}, DelayAction{0},
          ProfiledDriveAction{CELL_SIZE_METERS - 0.01, theta, EXPLORE_SPEED.maxSpeed,
                              EXPLORE_SPEED.maxSpeed}));
    }
    // Explore (slow) variants
    if (cls == EX_FWD0) {
      return makeFwdAction(arg, io, s, EXPLORE_SPEED);
    }
    if (cls == EX_ST0) {
      return makeCurveAction(arg, io, s, EXPLORE_SPEED);
    }
    // Fast variants
    if (cls == FWD0) {
      return makeFwdAction(arg, io, s, FAST_SPEED);
    }
    if (cls == ST0) {
      return makeCurveAction(arg, io, s, FAST_SPEED);
    }
    return std::make_unique<EmptyAction>();
  }

  void runMMS(MouseState &s, MouseIO &io) {
    io.update(s);
    unsigned char c = buf[pc++];

    unsigned char cls = c & 0b11100000;
    unsigned char arg = c & 0b00011111;
    if (c == STOP) {
      canceled = true;
      return;
    }

    if (cls == EX_FWD0 || cls == FWD0) {
      for (int i = 0; i < arg; ++i) {
        GridCoord v = dirToVector(s.dir);
        IdealState next{GridCoord{s.x + v.x, s.y + v.y}};
        io.setState(next);
      }
      return;
    }
    if (c == EX_ST90L || c == ST90L) {
      GridCoord v = dirToVector((unsigned char)LCIRC4(s.dir));
      IdealState next{GridCoord{s.x + v.x, s.y + v.y}};
      io.setState(next);
      return;
    }
    if (c == IPT180) {
      GridCoord v = dirToVector((unsigned char)LCIRC4(LCIRC4(s.dir)));
      IdealState next{GridCoord{s.x + v.x, s.y + v.y}};
      io.setState(next);
      return;
    }
    if (c == EX_ST90R || c == ST90R) {
      GridCoord v = dirToVector((unsigned char)RCIRC4(s.dir));
      IdealState next{GridCoord{s.x + v.x, s.y + v.y}};
      io.setState(next);
      return;
    }

    if (cls == DFWD0) {
      return;
    }
  }
};
