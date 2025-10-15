#include <vector>

#include "Action.h"
#include "CommandGenerator.h"
#include "Commands.h"
struct CommandAction : Action {
  std::vector<unsigned char> buf;
  size_t pc = 0;
  bool canceled = false;

  void load(std::vector<unsigned char> b) {
    buf = std::move(b);
    canceled = false;
    pc = 0;
  }
  void cancel() override { canceled = true; }
  bool completed() const override { return canceled || pc >= buf.size(); }

  void run(MouseState& s, MouseIO& io) override {
    if (completed()) return;
    runMMS(s, io);
  }
  void runMMS(MouseState& s, MouseIO& io) {
    unsigned char c = buf[pc++];

    unsigned char cls = c & 0b11100000;
    unsigned char arg = c & 0b00011111;
    // std::cerr << commandString({c}) << std::endl << std::endl;
    if (c == STOP) {
      canceled = true;
      return;
    }

    if (cls == FWD0) {
      int runs = arg;
      for (int i = 0; i < arg; ++i) {
        GridCoord v = dirToVector(s.dir);
        IdealState next{GridCoord{s.x + v.x, s.y + v.y}};
        io.setState(next);
      }
      return;
    }
    if (c == ST90L) {
      GridCoord v = dirToVector((unsigned char)LCIRC4(s.dir));
      IdealState next{GridCoord{s.x + v.x, s.y + v.y}};
      io.setState(next);
      return;
    }
    if (c == ST180) {
      GridCoord v = dirToVector((unsigned char)LCIRC4(LCIRC4(s.dir)));
      IdealState next{GridCoord{s.x + v.x, s.y + v.y}};
      io.setState(next);
      return;
    }
    if (c == ST90R) {
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
