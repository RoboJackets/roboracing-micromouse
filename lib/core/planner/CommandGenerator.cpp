#include "CommandGenerator.h"

// Transition function definitions
CommandGenerator::State CommandGenerator::on(Straight s, Move m) {
  if (m == Move::Forward) {
    return Straight{s.cells + 1};
  } else if (m == Move::Left) {
    push_cmd({
      Command::Type::Forward,
      s.cells
    });
    return Turning{Side::Left };
  } else if (m == Move::Right) {
    push_cmd({
      Command::Type::Forward,
      s.cells
    });
    return Turning{Side::Right};
  }
}

CommandGenerator::State CommandGenerator::on(Turning s, Move m) {
  if (s.side == Side::Left) {
    if (m == Move::Forward) {
      push_cmd({
        Command::Type::SmoothTurn,
        -2
      });
      return Straight{1};
    } else if (m == Move::Left) {
      push_cmd({
        Command::Type::SmoothTurn,
        -2
      });
      return Turning{Side::Left };
    } else if (m == Move::Right) {
      push_cmd({
        Command::Type::SmoothTurn,
        -2
      });
      return Turning{Side::Right };
    }
  } else if (s.side == Side::Right) {
    if (m == Move::Forward) {
      push_cmd({
        Command::Type::SmoothTurn,
        2
      });
      return Straight{1};
    } else if (m == Move::Left) {
      push_cmd({
        Command::Type::SmoothTurn,
        2
      });
      return Turning{Side::Left };
    } else if (m == Move::Right) {
      push_cmd({
        Command::Type::SmoothTurn,
        2
      });
      return Turning{Side::Right };
    }
  }
}
