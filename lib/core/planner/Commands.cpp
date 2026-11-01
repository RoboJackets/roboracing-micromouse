#include "Commands.h"

#include "actions/ControlActions.h"
#include "maze/Maze.h"
#include "robot/Robot.h"

// Transition function definitions
CommandGenerator::State CommandGenerator::on(Straight s, Move m) {
  if (m == Move::Forward) {
    return Straight{s.cells + 1};
  } else if (m == Move::Left) {
    if (s.cells > 0) {
      push_cmd({
        Command::Type::Forward,
        s.cells
      });
    }
    return Turning{Side::Left };
  } else if (m == Move::Right) {
    if (s.cells > 0) {
      push_cmd({
        Command::Type::Forward,
        s.cells
      });
    }
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
  } else if (m == Move::Back) {
    if (s.cells > 0) {
      push_cmd({
        Command::Type::Forward,
        s.cells
      });
    }
    push_cmd({
      Command::Type::InPlaceTurn,
      4
    });
    return Straight{1};
  } else if (m == Move::Continue) {
    if (s.cells > 0) {
      push_cmd({
        Command::Type::Forward,
        s.cells
      });
    }
    return Straight{};
  }
  return state;
}

CommandGenerator::State CommandGenerator::on(Turning s, Move m) {
  push_cmd({
    Command::Type::SmoothTurn,
    s.side == Side::Left ? -2 : 2
  });
  // The turn finishes inside the cell it started in, so the cell the turn move
  // entered still has to be driven.
  if (m == Move::Forward) {
    return Straight{2};
  } else if (m == Move::Left || m == Move::Right) {
    push_cmd({
      Command::Type::Forward,
      1
    });
    return Turning{m == Move::Left ? Side::Left : Side::Right};
  } else if (m == Move::Stop) {
    push_cmd({
      Command::Type::Forward,
      1
    });
    push_cmd({
      Command::Type::Stop,
      0
    });
    return Straight{};
  } else if (m == Move::Back) {
    push_cmd({
      Command::Type::Forward,
      1
    });
    push_cmd({
      Command::Type::InPlaceTurn,
      4
    });
    return Straight{1};
  }
  // Continue: explore replans from wherever the turn leaves the robot
  return Straight{};
}

std::unique_ptr<Action> CommandTranslator::translate(const Command& c, Robot& r, const SpeedProfile& sp) {
  switch (c.type) {
    case Command::Type::Forward:
      return makeFwdAction(c.amt, r, sp);
    case Command::Type::SmoothTurn:
      return makeCurveAction(c.amt, r, sp);
    case Command::Type::InPlaceTurn:
      return makeInPlaceTurnAction(c.amt, r, sp);
    case Command::Type::Stop:
      break;
  }
  return std::make_unique<StopAction>();
}

std::unique_ptr<Action> CommandTranslator::makeFwdAction(int8_t amt, Robot &r, const SpeedProfile &sp) {
  const Cell v = octantOffset(angle);
  const Cell nextCell = cell + v * amt;
  const Translation2d nextPos = cellEdge(nextCell, (angle + 4) % 8);
  Pose2d nextPose = pose;
  nextPose.xy = nextPos;

  std::unique_ptr<Action> action = std::make_unique<ProfiledDriveAction>(
    pose.xy,
    nextPos,
    TrapezoidalProfile::Constraints{
      sp.maxSpeed,
      MAX_ACCEL_M_S2 * DRIVE_ACCEL_SCALE
    }
  );

  pose.xy = nextPos;
  cell = nextCell;
  return action;
}

std::unique_ptr<Action> CommandTranslator::makeCurveAction(int8_t amt, Robot& r, const SpeedProfile& sp) {
  // TODO: real smooth turn. Until ProfiledCurveAction plans from absolute
  // geometry, pivot at the cell center; this leaves the translator in the same
  // state the generator expects after a smooth turn (inside the same cell).
  return makeInPlaceTurnAction(amt, r, sp);
}

std::unique_ptr<Action> CommandTranslator::makeInPlaceTurnAction(int8_t amt, Robot& r, const SpeedProfile& sp) {
  angle = normalizeOctant(angle + amt);
  
  const Translation2d nextPos = cellCenter(cell);
  
  double target = M_PI / 2.0 - angle * M_PI / 4.0;

  std::unique_ptr<Action> drive = std::make_unique<ProfiledDriveAction>(
    pose.xy,
    nextPos,
    TrapezoidalProfile::Constraints{
      sp.maxSpeed,
      MAX_ACCEL_M_S2 * DRIVE_ACCEL_SCALE
    }
  );

  std::unique_ptr<Action> turn = std::make_unique<ProfiledRotationAction>(
    target
  );

  std::vector<std::unique_ptr<Action>> steps;
  steps.push_back(std::move(drive));
  steps.push_back(std::move(turn));
  std::unique_ptr<Action> action = std::make_unique<SequentialAction>(std::move(steps));

  pose.theta = target;
  pose.xy = nextPos;
  
  return action;
}
