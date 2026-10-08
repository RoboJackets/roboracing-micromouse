#include "Commands.h"

#include "actions/ControlActions.h"
#include "maze/Maze.h"
#include "robot/Robot.h"

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
  } else if (m == Move::Stop) {
    if (s.cells > 0) {
      push_cmd({
        Command::Type::Forward,
        s.cells
      });
    }
    push_cmd({
      Command::Type::Stop,
      0
    });
    return Straight{};
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
    } else if (m == Move::Stop) {
      push_cmd({
        Command::Type::SmoothTurn,
        -2
      });
      push_cmd({
        Command::Type::Stop,
        0
      });
      return Straight{};
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
    } else if (m == Move::Stop) {
      push_cmd({
        Command::Type::SmoothTurn,
        2
      });
      push_cmd({
        Command::Type::Stop,
        0
      });
      return Straight{};
    }
  }
}

std::unique_ptr<Action> CommandTranslator::makeFwdAction(int8_t amt, Robot &r, const SpeedProfile &sp) {
  const Cell v = octantOffset(goalAngle);
  const WorldCoord pose = r.getWorldCoord();
  const WorldCoord rel = cellRelative(pose, cellOf(pose));

  double halfCell = CELL_SIZE_METERS / 2.0;
  double dx = v.x != 0 ? (v.x * amt * CELL_SIZE_METERS + halfCell - rel.x -
                          v.x * CELL_STOP_SHORT)
                       : 0;
  double dy = v.y != 0 ? (v.y * amt * CELL_SIZE_METERS + halfCell - rel.y -
                          v.y * CELL_STOP_SHORT)
                       : 0;

  double distance = std::sqrt(dx * dx + dy * dy);
  double travelAngle = M_PI / 2.0 - goalAngle * M_PI / 4.0;
  return std::make_unique<SequentialAction>(
      SequentialAction::make(ProfiledDriveAction{
          distance, travelAngle, sp.driveFinalVelocity, sp.maxSpeed}));
}

std::unique_ptr<Action> CommandTranslator::makeCurveAction(int8_t amt, Robot& r, const SpeedProfile& sp) {
    goalAngle += amt;
    goalAngle = normalizeOctant(goalAngle);

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
