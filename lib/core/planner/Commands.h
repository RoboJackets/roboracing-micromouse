#pragma once

#include <sstream>
#include <string>
#include <vector>
#include "actions/Action.h"
#include "maze/Maze.h"
#include "Constants.h"
#include "Tuning.h"
#include <cmath>
#include <variant>
#include <memory>
#include <iostream>

enum class Move {
  Forward,
  Left,
  Right,
  Back,
  Stop,
  Continue
};

enum class Side {
  // ensure that the left and right for move and side can be casted to the same int
  // makes comparisons much easier later
  Left = static_cast<int>(Move::Left), 
  Right = static_cast<int>(Move::Right)
};

struct Command {
  // most basic level of commands
  // future can optimize a diagonal enter and exit if 135 etc.
  enum class Type {
    STRAIGHT_FORWARD,
    DIAG_FORWARD,
    LEFT_SMOOTH_90,
    RIGHT_SMOOTH_90,
    DIAG_TURN_LEFT_ENTER,
    DIAG_TURN_LEFT_EXIT,
    DIAG_TURN_RIGHT_ENTER,
    DIAG_TURN_RIGHT_EXIT,
    IN_PLACE_180,
    STOP
  };
  Type type;
  // only need distance for variations of forwards commands
  // unit is determined by type of forward
  int distance = -1;

  void print() const {
    switch (type) {
      case Type::STRAIGHT_FORWARD:
        std::cout << "STRAIGHT_FORWARD, distance: " << distance << "\n";
        break;
      case Type::DIAG_FORWARD:
        std::cout << "DIAG_FORWARD, distance: " << distance << "\n";
        break;
      case Type::LEFT_SMOOTH_90:
        std::cout << "LEFT_SMOOTH_90" << "\n";
        break;
      case Type::RIGHT_SMOOTH_90:
        std::cout << "RIGHT_SMOOTH_90" << "\n";
        break;
      case Type::DIAG_TURN_LEFT_ENTER:
        std::cout << "DIAG_TURN_LEFT_ENTER" << "\n";
        break;
      case Type::DIAG_TURN_LEFT_EXIT:
        std::cout << "DIAG_TURN_LEFT_EXIT" << "\n";
        break;
      case Type::DIAG_TURN_RIGHT_ENTER:
        std::cout << "DIAG_TURN_RIGHT_ENTER" << "\n";
        break;
      case Type::DIAG_TURN_RIGHT_EXIT:
        std::cout << "DIAG_TURN_RIGHT_EXIT" << "\n";
        break;
      case Type::IN_PLACE_180:
        std::cout << "IN_PLACE_180" << "\n";
        break;
      case Type::STOP:
        std::cout << "STOP" << "\n";
        break;
    }
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

  // just need to keep track of how many cells straight
  struct Straight {
    int cells = 0;
  };

  // need to keep track of what side we are turning too
  struct Turning {
    Side side;
  };

  // need to keep track of 
  // last side we turned to as we are traversing the diagonal
  // what the first turn was, i.e. overall diagonal side
  // need to keep track of how many diagonal forwards we need to add at the end
  struct Diagonal {
    Side lastSide;
    int cells = 0;
  };

  using State = std::variant<Straight, Turning, Diagonal>;

  // Transitions
  // state' <- f(state, move)
  State on(Straight s, Move m);
  State on(Turning s, Move m);
  State on(Diagonal s, Move m);

  // Logistics
  void push_cmd(Command c) {
    out.push_back(c);
  }
  
  State state = Straight{};
  std::vector<Command> out;
};

struct CommandTranslator {
  int goalAngle = 0;
  Cell goalCell{};

  std::unique_ptr<Action> translate(const Command& c, Robot& r, const SpeedProfile& sp);

private:
  // all actions ensure that robot starts at beginning of next cell, as in the starting position
  // diagonal straights must be surrounded by diagonal turns

  // this is a forward action in the "straight" direction
  // since walls are perpendicular to side IR sensors,
  // IR can be used for pose correction as well
  std::unique_ptr<Action> makeFwdActionStraight(int8_t amt, Robot &r,
                                        const SpeedProfile &sp);

  // this is a forward action in the diagonal direction
  // includes only gyro and odometry level position correction
  std::unique_ptr<Action> makeFwdActionDiagonal(int8_t amt, Robot &r,
                                        const SpeedProfile &sp);

  // stand smooth curve action
  // starts at front of a cell and ends at the start of another cell     
  std::unique_ptr<Action> makeSmoothCurveAction(int8_t amt, Robot &r,
                                          const SpeedProfile &sp);

  // smooth curve action for diagonals
  // handles entering and exiting diagonals, ensuring everything connects
  std::unique_ptr<Action> makeSmoothDiagCurveActionEnter(int8_t amt, Robot &r,
                                          const SpeedProfile &sp);

  std::unique_ptr<Action> makeSmoothDiagCurveActionExit(int8_t amt, Robot &r,
                                          const SpeedProfile &sp);


  // in place is only needed for explore, usually just for 180 deg turns
  std::unique_ptr<Action> makeInPlaceTurnAction(int8_t amt, Robot &r,
                                                const SpeedProfile &sp);
};

struct DeferredCommandAction : Action {
  Command c;
  CommandTranslator &translator;
  SpeedProfile sp;
  std::unique_ptr<Action> inner;

  DeferredCommandAction(Command c, CommandTranslator &translator, SpeedProfile sp)
      : c(c), translator(translator), sp(sp) {}

  void begin(Robot &r) override {
    inner = translator.translate(c, r, sp);
    inner->begin(r);
  }
  void run(Robot &r) override { inner->run(r); }
  void end(Robot &r) override {
    if (inner) inner->end(r);
  }
  void cancel() override {
    canceled = true;
    if (inner) inner->cancel();
  }
  bool completed() const override {
    return canceled || (inner && inner->completed());
  }
};
