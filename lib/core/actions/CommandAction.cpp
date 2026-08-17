#include "CommandAction.h"

#include "maze/Maze.h"
#include "robot/Robot.h"

void CommandAction::load(std::vector<unsigned char> b) {
  buf = std::move(b);
  canceled = false;
  pc = 0;
  curr.reset();
}

void CommandAction::cancel() {
  canceled = true;
  if (curr)
    curr->cancel();
}

void CommandAction::end(Robot &r) {
  if (curr)
    curr->end(r);
}

void CommandAction::run(Robot &r) {
  if (completed())
    return;
  if (!curr) {
    curr = determineAction(r);
  }
  curr->run(r);
  if (curr->completed()) {
    r.observe();
    curr->end(r);
    curr.reset();
  }
}

int CommandAction::turnAmount(unsigned char arg) {
  if (arg == 0)
    return 0;
  int magnitude = 0;
  unsigned char lower = arg & 0b00000111;
  if (lower == 1)
    magnitude = 1; // 45
  else if (lower == 2)
    magnitude = 2; // 90
  else if (lower == 4)
    magnitude = 3; // 135
  bool right = (arg & 0b00010000) != 0;
  return right ? magnitude : -magnitude;
}

std::unique_ptr<Action> CommandAction::makeFwdAction(unsigned char arg,
                                                     Robot &r,
                                                     const SpeedProfile &sp) {
  const Cell v = octantOffset(goalAngle);
  const WorldCoord pose = r.getWorldCoord();
  const WorldCoord rel = cellRelative(pose, cellOf(pose));

  double halfCell = CELL_SIZE_METERS / 2.0;
  double dx = v.x != 0 ? (v.x * arg * CELL_SIZE_METERS + halfCell - rel.x -
                          v.x * CELL_STOP_SHORT)
                       : 0;
  double dy = v.y != 0 ? (v.y * arg * CELL_SIZE_METERS + halfCell - rel.y -
                          v.y * CELL_STOP_SHORT)
                       : 0;

  double distance = std::sqrt(dx * dx + dy * dy);
  double travelAngle = M_PI / 2.0 - goalAngle * M_PI / 4.0;
  return std::make_unique<SequentialAction>(
      SequentialAction::make(ProfiledDriveAction{
          distance, travelAngle, sp.driveFinalVelocity, sp.maxSpeed}));
}

std::unique_ptr<Action> CommandAction::makeCurveAction(unsigned char arg,
                                                       Robot &r,
                                                       const SpeedProfile &sp) {
  goalAngle += turnAmount(arg);
  double targetTheta = M_PI / 2.0 - goalAngle * M_PI / 4.0;
  double currentTheta = r.getWorldCoord().theta;
  double turnAngle = std::atan2(std::sin(targetTheta - currentTheta),
                                std::cos(targetTheta - currentTheta));

  goalAngle = normalizeOctant(goalAngle);

  double travelAngle = M_PI / 2.0 - goalAngle * M_PI / 4.0;
  return std::make_unique<SequentialAction>(SequentialAction::make(
      ProfiledCurveAction(sp.curveRadius, turnAngle, sp.curveFinalVelocity,
                          sp.maxSpeed),
      ProfiledDriveAction{sp.curveTrailDistance, travelAngle,
                          sp.driveFinalVelocity, sp.maxSpeed}));
}

std::unique_ptr<Action> CommandAction::determineAction(Robot &r) {
  unsigned char c = buf[pc++];

  unsigned char cls = c & 0b11100000;
  unsigned char arg = c & 0b00011111;
  if (c == STOP) {
    r.driveDuty(0, 0);
    canceled = true;
    return std::make_unique<EmptyAction>();
  }
  if (c == IPT180) {
    goalAngle = normalizeOctant(goalAngle + 4);
    r.driveDuty(0, 0);
    double theta = M_PI / 2.0 - goalAngle * M_PI / 4.0;
    double currentTheta = r.getWorldCoord().theta;
    double turnAngle = std::atan2(std::sin(theta - currentTheta),
                                  std::cos(theta - currentTheta));
    return std::make_unique<SequentialAction>(SequentialAction::make(
        ProfiledRotationAction{turnAngle}, DelayAction{0},
        ProfiledDriveAction{CELL_SIZE_METERS - CELL_STOP_SHORT, theta,
                            EXPLORE_SPEED.maxSpeed, EXPLORE_SPEED.maxSpeed}));
  }
  // Explore (slow) variants
  if (cls == EX_FWD0) {
    return makeFwdAction(arg, r, EXPLORE_SPEED);
  }
  if (cls == EX_ST0) {
    return makeCurveAction(arg, r, EXPLORE_SPEED);
  }
  // Fast variants
  if (cls == FWD0) {
    return makeFwdAction(arg, r, FAST_SPEED);
  }
  if (cls == ST0) {
    return makeCurveAction(arg, r, FAST_SPEED);
  }
  return std::make_unique<EmptyAction>();
}
