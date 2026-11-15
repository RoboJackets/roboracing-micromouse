#include "Commands.h"

#include "actions/ControlActions.h"
#include "maze/Maze.h"
#include "robot/Robot.h"

// Transition function definitions

CommandGenerator::State CommandGenerator::on(Straight s, Move m) {
  switch (m) {
    case Move::Forward:
      return Straight{s.cells + 1};

    case Move::Left:
      push_cmd({
        Command::Type::STRAIGHT_FORWARD,
        s.cells
      });
      return Turning{Side::Left};

    case Move::Right:
      push_cmd({
        Command::Type::STRAIGHT_FORWARD,
        s.cells
      });
      return Turning{Side::Right};

    case Move::Back:
      if (s.cells > 0) {
        push_cmd({
          Command::Type::STRAIGHT_FORWARD,
          s.cells
        });
      }
      push_cmd({
        Command::Type::IN_PLACE_180,
      });
      // this will still be the same amount as a normal straight
      // kind of assuming that we are doing the press against back wall for pose correction
      // so return to that "in the beginning of the current cell" position
      return Straight{1};

    case Move::Stop:
      if (s.cells > 0) {
        push_cmd({
          Command::Type::STRAIGHT_FORWARD,
          s.cells
        });
      }
      push_cmd({
        Command::Type::STOP
      });
      return Straight{};

    case Move::Continue:
      if (s.cells > 0) {
        push_cmd({
          Command::Type::STRAIGHT_FORWARD,
          s.cells
        });
      }
      return Straight{};
    default: 
      return state;
  }
}

CommandGenerator::State CommandGenerator::on(Turning s, Move m) {
  // defining if left or right now so can handle both cases in same switch
  Command turn = (s.side == Side::Left) 
    ? Command{Command::Type::LEFT_SMOOTH_90} 
    : Command{Command::Type::RIGHT_SMOOTH_90};
  Command enter_diagonal = (s.side == Side::Left) 
    ? Command{Command::Type::DIAG_TURN_LEFT_ENTER} 
    : Command{Command::Type::DIAG_TURN_RIGHT_ENTER};

  switch(m) {
    case Move::Forward:
      push_cmd(turn);
      return Straight{1};

    case Move::Left:
    case Move::Right: {
      // Move and Side both have a left and right
      // Turn the current side to a move type to make comparison easier
      Move side_to_move = static_cast<Move>(static_cast<int>(s.side));
      Side move_to_side = static_cast<Side>(static_cast<int>(m));

      if (side_to_move == m) {
        push_cmd(turn);
        return Turning{move_to_side};
      } else {
        push_cmd(enter_diagonal);
        return Diagonal{move_to_side, 0};
      }
    }
    case Move::Stop:
      push_cmd(turn);
      push_cmd({Command::Type::STOP});
      return Straight{};

    case Move::Back:
      push_cmd(turn);
      push_cmd({Command::Type::IN_PLACE_180});
      return Straight{1};

    case Move::Continue:
      push_cmd(turn);
      return Straight{};
    
    default:
      return state;
  }
} 

CommandGenerator::State CommandGenerator::on(Diagonal s, Move m) {
  // defining if left or right now so can handle both cases in same switch
  Command turn = (s.lastSide == Side::Left)
    ? Command{Command::Type::DIAG_TURN_LEFT_EXIT} 
    : Command{Command::Type::DIAG_TURN_RIGHT_EXIT};

  switch(m) {
    case Move::Forward:
      push_cmd({Command::Type::DIAG_FORWARD, s.cells});
      push_cmd(turn);
      return Straight{1};

    case Move::Left:
    case Move::Right: {
      // Move and Side both have a left and right
      // Turn the current side to a move type to make comparison easier
      Move side_to_move = static_cast<Move>(static_cast<int>(s.lastSide));
      Side move_to_side = static_cast<Side>(static_cast<int>(m));

      if (side_to_move == m) {
        push_cmd({Command::Type::DIAG_FORWARD, s.cells});
        push_cmd(turn);
      } else {
        return Diagonal{move_to_side, s.cells + 1};
      }
    }
    case Move::Stop:
      push_cmd({Command::Type::DIAG_FORWARD, s.cells});
      push_cmd({Command::Type::STOP});
      return Straight{};

    case Move::Continue:
      push_cmd({Command::Type::DIAG_FORWARD, s.cells});
      return Straight{};
    
    // back should never be entered in a diagonal...
    case Move::Back:
    default:
      return state;
  }
}

std::unique_ptr<Action> CommandTranslator::translate(const Command& c, Robot& r, const SpeedProfile& sp) {
  switch (c.type) {
    case Command::Type::STRAIGHT_FORWARD:
      return makeFwdActionStraight(c.distance, r, sp);

    case Command::Type::DIAG_FORWARD:
      return makeFwdActionDiagonal(c.distance, r, sp);

    case Command::Type::LEFT_SMOOTH_90:
      return makeSmoothCurveAction(-2.0, r, sp);

    case Command::Type::RIGHT_SMOOTH_90:
      return makeSmoothCurveAction(2.0, r, sp);

    case Command::Type::DIAG_TURN_LEFT_ENTER:
      return makeSmoothDiagCurveAction(-1.0, r, sp);

    case Command::Type::DIAG_TURN_RIGHT_ENTER:
      return makeSmoothDiagCurveAction(1.0, r, sp);

    case Command::Type::DIAG_TURN_LEFT_EXIT:
      return makeSmoothDiagCurveAction(-1.0, r, sp);

    case Command::Type::DIAG_TURN_RIGHT_EXIT:
      return makeSmoothDiagCurveAction(1.0, r, sp);

    case Command::Type::IN_PLACE_180:
      return makeInPlaceTurnAction(4, r, sp);

    case Command::Type::STOP:
    default:
      return std::make_unique<StopAction>();
  }
}

std::unique_ptr<Action> CommandTranslator::makeFwdActionStraight(int8_t amt, Robot &r, const SpeedProfile &sp) {
  const Cell v = octantOffset(goalAngle);
  const WorldCoord pose = r.getWorldCoord();
  const WorldCoord rel = cellRelative(pose, goalCell);

  double halfCell = CELL_SIZE_METERS / 2.0;
  double dx = v.x * amt * CELL_SIZE_METERS + halfCell - rel.x - v.x * CELL_STOP_SHORT;
  double dy = v.y * amt * CELL_SIZE_METERS + halfCell - rel.y - v.y * CELL_STOP_SHORT;

  double distance = (dx * v.x + dy * v.y) / std::hypot(v.x, v.y);
  double travelAngle = M_PI / 2.0 - goalAngle * M_PI / 4.0;
  goalCell = Cell{goalCell.x + v.x * amt, goalCell.y + v.y * amt};
  return std::make_unique<SequentialAction>(
      SequentialAction::make(ProfiledDriveAction{
          distance, travelAngle, sp.driveFinalVelocity, sp.maxSpeed}));
}

std::unique_ptr<Action> CommandTranslator::makeFwdActionDiagonal(int8_t amt, Robot &r, const SpeedProfile &sp) {
  return 0;
}

std::unique_ptr<Action> CommandTranslator::makeSmoothCurveAction(int8_t amt, Robot& r, const SpeedProfile& sp) {
    goalAngle += amt;
    goalAngle = normalizeOctant(goalAngle);
    const Cell v = octantOffset(goalAngle);
    goalCell = Cell{goalCell.x + v.x, goalCell.y + v.y};

    double targetTheta = M_PI / 2.0 - goalAngle * M_PI / 4.0;
    double currentTheta = r.getWorldCoord().theta;
    double turnAngle = std::atan2(std::sin(targetTheta - currentTheta),
                                    std::cos(targetTheta - currentTheta));


    double travelAngle = M_PI / 2.0 - goalAngle * M_PI / 4.0;
    return std::make_unique<SequentialAction>(SequentialAction::make(
        ProfiledCurveAction(sp.curveRadius, turnAngle, sp.curveFinalVelocity,
                            sp.maxSpeed),
        ProfiledDriveAction{sp.curveTrailDistance, travelAngle,
                            sp.driveFinalVelocity, sp.maxSpeed}));
}

std::unique_ptr<Action> CommandTranslator::makeSmoothDiagCurveActionEnter(int8_t amt, Robot& r, const SpeedProfile& sp) {
  return 0;
}

std::unique_ptr<Action> CommandTranslator::makeSmoothDiagCurveActionExit(int8_t amt, Robot& r, const SpeedProfile& sp) {
  return 0;
}

std::unique_ptr<Action> CommandTranslator::makeInPlaceTurnAction(int8_t amt, Robot& r, const SpeedProfile& sp) {
    const Cell v = octantOffset(goalAngle);
    const WorldCoord pose = r.getWorldCoord();
    const WorldCoord rel = cellRelative(pose, goalCell);

    double halfCell = CELL_SIZE_METERS / 2.0;
    double centerDistance = ((halfCell - rel.x) * v.x + (halfCell - rel.y) * v.y) /
                            std::hypot(v.x, v.y);
    double travelAngle = M_PI / 2.0 - goalAngle * M_PI / 4.0;

    goalAngle += amt;
    goalAngle = normalizeOctant(goalAngle);

    double targetTheta = M_PI / 2.0 - goalAngle * M_PI / 4.0;
    double turnAngle = std::atan2(std::sin(targetTheta - pose.theta),
                                  std::cos(targetTheta - pose.theta));

    return std::make_unique<SequentialAction>(SequentialAction::make(
        ProfiledDriveAction{centerDistance, travelAngle, 0, sp.maxSpeed},
        ProfiledRotationAction{turnAngle}));
}
