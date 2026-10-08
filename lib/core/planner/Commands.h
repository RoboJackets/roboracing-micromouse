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

// The input to the CommandGenerator is a sequence of Moves
enum class Move {
  Forward, Left, Right, Back, Stop
};

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
  enum class Type { Forward, SmoothTurn, Stop };

  Type type;
  int8_t amt; // number of cells if Forward, signed multiple of 45 degrees otherwise
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
    int8_t cells = 0;
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

struct CommandTranslator {
  int goalAngle = 0;

  std::unique_ptr<Action> translate(const Command& c, Robot& r, const SpeedProfile& sp);

private:
  std::unique_ptr<Action> makeFwdAction(int8_t amt, Robot &r,
                                        const SpeedProfile &sp);

  std::unique_ptr<Action> makeCurveAction(int8_t amt, Robot &r,
                                          const SpeedProfile &sp);
};
