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
    // Serial.println("running");
    if (completed())
      return;
    if (io.isMMS()) {
      runMMS(s, io);
      return;
    }
    if (!curr) {
      // Serial.println("BANG");
      curr = determineAction(s, io);
    }
    curr->run(s, io);
    if (curr->completed()) {
      curr->end(s, io);
      s.x = io.getGridCoord().x;
      s.y = io.getGridCoord().y;
      s.dir = io.getGridCoord().dir;
      // Serial.println("MEOW");
      curr.reset();
    }
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
    if (cls == EX_FWD0) {
      GridCoord v = angleToVector(goalAngle);
      goal.x += v.x * arg;
      goal.y += v.y * arg;

      WorldCoord rel = io.getWorldCoord().gridRelativeCoords(io.getGridCoord());
      double halfCell = CELL_SIZE_METERS / 2.0;
      double dx =
          v.x != 0 ? (v.x * arg * CELL_SIZE_METERS + halfCell - rel.x - v.x * 0.03)
                   : 0;
      double dy =
          v.y != 0 ? (v.y * arg * CELL_SIZE_METERS + halfCell - rel.y - v.y * 0.03)
                   : 0;

      double distance = std::sqrt(dx * dx + dy * dy);
      double travelAngle =
          M_PI / 2.0 -
          goalAngle * M_PI / 4.0; // convert from 0 -> up to 0 -> right
      Serial.printf("FWD%d    WALL: %d\n", arg,
                    s.walls[io.getGridCoord().x][io.getGridCoord().y]);
      return std::make_unique<SequentialAction>(SequentialAction::make(
          ProfiledDriveAction{distance, travelAngle, 0.1}));
    }
    if (c == IPT180) {
      goalAngle += 4;
      goalAngle = (goalAngle + 8) % 8;
      io.driveVoltage(0, 0);
      double theta = M_PI / 2.0 - goalAngle * M_PI / 4.0;
      return std::make_unique<SequentialAction>(SequentialAction::make(
          ProfiledRotationAction{theta},
          ProfiledDriveAction{CELL_SIZE_METERS, theta, 0.1}));
    }
    if (cls == EX_ST0) {
      Serial.printf("CURVE    WALL: %d\n",
                    s.walls[io.getGridCoord().x][io.getGridCoord().y]);
      if (c == EX_ST45L) {
        goalAngle -= 1;
      } else if (c == EX_ST90L) {
        goalAngle -= 2;
      } else if (c == EX_ST135L) {
        goalAngle -= 3;
      } else if (c == EX_ST45R) {
        goalAngle += 1;
      } else if (c == EX_ST90R) {
        goalAngle += 2;
      } else if (c == EX_ST135R) {
        goalAngle += 3;
      }
      double targetTheta = M_PI / 2.0 - goalAngle * M_PI / 4.0;
      double currentTheta = io.getWorldCoord().theta;
      double turnAngle = std::atan2(std::sin(targetTheta - currentTheta),
                                    std::cos(targetTheta - currentTheta));

      // Might need to be adjusted so it takes position into consideration,
      // not just angle.

      goalAngle = (goalAngle + 8) % 8;

      GridCoord v = angleToVector(goalAngle);
      goal.x += v.x;
      goal.y += v.y;
      double travelAngle = M_PI / 2.0 - goalAngle * M_PI / 4.0;
      return std::make_unique<SequentialAction>(
          SequentialAction::make(ProfiledCurveAction(0.03, turnAngle, 0.1),
                                 ProfiledDriveAction{0.01, travelAngle, 0.1}));
    }
    return std::make_unique<EmptyAction>();
  }
  void runMMS(MouseState &s, MouseIO &io) {
    io.update(s);
    unsigned char c = buf[pc++];

    unsigned char cls = c & 0b11100000;
    unsigned char arg = c & 0b00011111;
    // std::cerr << commandString({c}) << std::endl << std::endl;
    if (c == STOP) {
      canceled = true;
      return;
    }

    if (cls == EX_FWD0) {
      int runs = arg;
      for (int i = 0; i < arg; ++i) {
        GridCoord v = dirToVector(s.dir);
        IdealState next{GridCoord{s.x + v.x, s.y + v.y}};
        io.setState(next);
      }
      return;
    }
    if (c == EX_ST90L) {
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
    if (c == EX_ST90R) {
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
