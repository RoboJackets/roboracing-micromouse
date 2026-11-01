#pragma once

#include <sstream>
#include <string>
#include <vector>
#include "actions/Action.h"
#include "Constants.h"
#include "Tuning.h"
#include <cmath>
#include <variant>
#include <memory>
#include "maze/Maze.h"

// The input to the CommandGenerator is a sequence of Moves
enum class Move {
  Forward, Left, Right, Back, Stop, Continue
};

// for debugging
inline char to_char(Move m) {
  switch (m) {
    case Move::Forward: return 'F';
    case Move::Left: return 'L';
    case Move::Right: return 'R';
    case Move::Back: return 'B';
    case Move::Stop: return 'S';
    case Move::Continue: return 'C';
  }
  return '?';
}

// Data types for the CommandGenerator state machine
enum ActionState {
  START,
  ORTHO_F,
  ORTHO_L,
  ORTHO_R,
  DIAG_LR,
  DIAG_RL,
  E_DIAG_LR,
  E_DIAG_RL,
  END
};

struct State {
  ActionState action = START;
  int x = 0;
  int y = 0;
};

enum class Side {
  Left = -1, Right = +1
};

// The output of the CommandGenerator.
// Represents a smooth path formed from the input moves
struct Command {
  enum class Type { Forward, SmoothTurn, Stop, InPlaceTurn };

  Type type;
  int8_t amt; // number of cells if Forward, signed multiple of 45 degrees otherwise

  Command(Type type, int amt) : type{type}, amt{static_cast<int8_t>(amt)} {
    
  }
};

class CommandGenerator {
public:
  std::vector<Command> exec(const std::vector<Move>& input) {
    state = Straight{};
    out.clear();
    for(Move m : input) {
      state = std::visit([&](auto s) { return on(s, m); }, state);
    }
    return out;
  }
private:
  // State definitions
  struct Straight {
    int cells = 0;
  };

  struct Turning {
    Side side;
  };

  using State = std::variant<Straight, Turning>;

  // Transitions
  // state' <- f(state, move)
  State on(Straight s, Move m);
  State on(Turning s, Move m);

  // Logistics
  void push_cmd(Command c) {
    out.push_back(c);
  }
  
  State state = Straight{};
  std::vector<Command> out;
};

class CommandTranslator {
  int angle = 0;
  Cell cell;
  Pose2d pose;

  std::unique_ptr<Action> makeFwdAction(int8_t amt, Robot &r,
                                        const SpeedProfile &sp);

  std::unique_ptr<Action> makeCurveAction(int8_t amt, Robot &r,
                                          const SpeedProfile &sp);

  std::unique_ptr<Action> makeInPlaceTurnAction(int8_t amt, Robot &r,
                                                const SpeedProfile &sp);
  
public:
  std::unique_ptr<Action> translate(const Command& c, Robot& r, const SpeedProfile& sp);

  void reset(Cell c, Pose2d p, int octant) { cell = c; pose = p; angle = octant; }
  Cell getCell() const { return cell; }
  Dir getFacing() const { return static_cast<Dir>(normalizeOctant(angle) / 2); }
};

// Translates its command when it begins rather than when it's built, so that
// each step of a multi-command route is planned from the robot's actual pose.
struct DeferredCommandAction : Action {
  CommandTranslator &translator;
  Command command;
  SpeedProfile speed;
  std::unique_ptr<Action> inner{};

  DeferredCommandAction(CommandTranslator &translator, Command command,
                        SpeedProfile speed)
      : translator(translator), command(command), speed(speed) {}

  void begin(Robot &r) override {
    inner = translator.translate(command, r, speed);
    inner->begin(r);
  }
  void run(Robot &r) override { inner->run(r); }
  void end(Robot &r) override {
    if (inner)
      inner->end(r);
  }
  void cancel() override {
    canceled = true;
    if (inner)
      inner->cancel();
  }
  bool completed() const override {
    return canceled || (inner && inner->completed());
  }
};
