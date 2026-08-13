#include "CommandAction.h"

void CommandAction::load(std::vector<unsigned char> b) {
  buf = std::move(b);
  canceled = false;
  pc = 0;
  curr.reset();
}

void CommandAction::run(MazeMap &map, MouseIO &io) {
  if (completed())
    return;
  if (!curr) {
    curr = determineAction(io);
  }
  curr->run(map, io);
  if (curr->completed()) {
    map.observe(io.getWorldCoord(), io.getRotationRate(), io.getSensorState());
    curr->end(map, io);
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
                                                     MouseIO &io,
                                                     const SpeedProfile &sp) {
  const Cell v = octantOffset(goalAngle);
  const WorldCoord pose = io.getWorldCoord();
  const WorldCoord rel = cellRelative(pose, cellOf(pose));

  double halfCell = CELL_SIZE_METERS / 2.0;
  double dx = v.x != 0
                  ? (v.x * arg * CELL_SIZE_METERS + halfCell - rel.x - v.x * 0.01)
                  : 0;
  double dy = v.y != 0
                  ? (v.y * arg * CELL_SIZE_METERS + halfCell - rel.y - v.y * 0.01)
                  : 0;

  double distance = std::sqrt(dx * dx + dy * dy);
  double travelAngle = M_PI / 2.0 - goalAngle * M_PI / 4.0;
  return std::make_unique<SequentialAction>(
      SequentialAction::make(ProfiledDriveAction{
          distance, travelAngle, sp.driveFinalVelocity, sp.maxSpeed}));
}

std::unique_ptr<Action> CommandAction::makeCurveAction(unsigned char arg,
                                                       MouseIO &io,
                                                       const SpeedProfile &sp) {
  goalAngle += turnAmount(arg);
  double targetTheta = M_PI / 2.0 - goalAngle * M_PI / 4.0;
  double currentTheta = io.getWorldCoord().theta;
  double turnAngle = std::atan2(std::sin(targetTheta - currentTheta),
                                std::cos(targetTheta - currentTheta));

  goalAngle = (goalAngle + 8) % 8;

  double travelAngle = M_PI / 2.0 - goalAngle * M_PI / 4.0;
  return std::make_unique<SequentialAction>(SequentialAction::make(
      ProfiledCurveAction(sp.curveRadius, turnAngle, sp.curveFinalVelocity,
                          sp.maxSpeed),
      ProfiledDriveAction{sp.curveTrailDistance, travelAngle,
                          sp.driveFinalVelocity, sp.maxSpeed}));
}

std::unique_ptr<Action> CommandAction::determineAction(MouseIO &io) {
  unsigned char c = buf[pc++];

  unsigned char cls = c & 0b11100000;
  unsigned char arg = c & 0b00011111;
  if (c == STOP) {
    io.driveVoltage(0, 0);
    canceled = true;
    return std::make_unique<EmptyAction>();
  }
  if (c == IPT180) {
    goalAngle += 4;
    goalAngle = (goalAngle + 8) % 8;
    io.driveVoltage(0, 0);
    double theta = M_PI / 2.0 - goalAngle * M_PI / 4.0;
    double currentTheta = io.getWorldCoord().theta;
    double turnAngle = std::atan2(std::sin(theta - currentTheta),
                                  std::cos(theta - currentTheta));
    return std::make_unique<SequentialAction>(SequentialAction::make(
        ProfiledRotationAction{turnAngle}, DelayAction{0},
        ProfiledDriveAction{CELL_SIZE_METERS - 0.01, theta,
                            EXPLORE_SPEED.maxSpeed, EXPLORE_SPEED.maxSpeed}));
  }
  // Explore (slow) variants
  if (cls == EX_FWD0) {
    return makeFwdAction(arg, io, EXPLORE_SPEED);
  }
  if (cls == EX_ST0) {
    return makeCurveAction(arg, io, EXPLORE_SPEED);
  }
  // Fast variants
  if (cls == FWD0) {
    return makeFwdAction(arg, io, FAST_SPEED);
  }
  if (cls == ST0) {
    return makeCurveAction(arg, io, FAST_SPEED);
  }
  return std::make_unique<EmptyAction>();
}
