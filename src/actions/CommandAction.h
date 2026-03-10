#pragma once
#include <vector>

#include "Action.h"
#include "CommandGenerator.h"
#include "Commands.h"
#include "MMSIO.h"
struct CommandAction : Action {
  std::vector<unsigned char> buf;
  size_t pc = 0;
  bool canceled = false;
  Action *curr = nullptr;
  GridCoord goal{};
  int goalAngle = 0;

  void load(std::vector<unsigned char> b) {
    buf = std::move(b);
    canceled = false;
    pc = 0;
    curr = nullptr;
  }
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled || pc >= buf.size(); }

  void run(MouseState &s, MouseIO &io) override {
    if (completed())
      return;
    if (io.isMMS()) {
      runMMS(s, io);
      return;
    }
    if (curr == nullptr) {
      determineAction(s, io);
    }
    curr->run(s, io);
    if (curr->completed()) {
      s.x = io.getGridCoord().x;
      s.y = io.getGridCoord().y;
      s.dir = io.getGridCoord().dir;
      curr = nullptr;
    }
  }
  void determineAction(MouseState &s, MouseIO &io) {
    unsigned char c = buf[pc++];

    unsigned char cls = c & 0b11100000;
    unsigned char arg = c & 0b00011111;
    if (cls == STOP) {
      io.driveVoltage(0, 0);
      canceled = true;
      return;
    }
    if (cls == EX_FWD0) {
      GridCoord v = angleToVector(goalAngle);
      goal.x += v.x * arg;
      goal.y += v.y * arg;
    }
    if (cls == EX_ST0) {
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
      goalAngle = (goalAngle + 8) % 8;

      GridCoord v = angleToVector(goalAngle);
      goal.x += v.x;
      goal.y += v.y;
    }
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
