#pragma once
#include <cstdint>

#include "Constants.h"
#include "Types.h"

constexpr int N = 16;
constexpr int INF = 300;

struct Cell {
  int x = 0;
  int y = 0;
};

constexpr bool operator==(Cell a, Cell b) { return a.x == b.x && a.y == b.y; }
constexpr bool operator!=(Cell a, Cell b) { return !(a == b); }

enum class Dir : uint8_t { North, East, South, West };

constexpr unsigned char TOP{0b1000};
constexpr unsigned char RIGHT{0b0100};
constexpr unsigned char DOWN{0b0010};
constexpr unsigned char LEFT{0b0001};

constexpr unsigned char wallBit(Dir d) {
  switch (d) {
  case Dir::North:
    return TOP;
  case Dir::East:
    return RIGHT;
  case Dir::South:
    return DOWN;
  case Dir::West:
    return LEFT;
  }
  return 0;
}

constexpr Dir opposite(Dir d) {
  return static_cast<Dir>((static_cast<int>(d) + 2) & 3);
}
constexpr Dir turnRight(Dir d) {
  return static_cast<Dir>((static_cast<int>(d) + 1) & 3);
}
constexpr Dir turnLeft(Dir d) {
  return static_cast<Dir>((static_cast<int>(d) + 3) & 3);
}

constexpr int turnCost(Dir from, Dir to) {
  const int rel = (static_cast<int>(to) - static_cast<int>(from)) & 3;
  return rel == 0 ? 0 : (rel == 2 ? 2 : 1);
}

constexpr Cell step(Cell c, Dir d) {
  switch (d) {
  case Dir::North:
    return {c.x, c.y + 1};
  case Dir::East:
    return {c.x + 1, c.y};
  case Dir::South:
    return {c.x, c.y - 1};
  case Dir::West:
    return {c.x - 1, c.y};
  }
  return c;
}

constexpr bool inBounds(Cell c) {
  return c.x >= 0 && c.x < N && c.y >= 0 && c.y < N;
}

inline constexpr Dir SCAN_ORDER[4]{Dir::North, Dir::West, Dir::South,
                                   Dir::East};

constexpr Cell octantOffset(int octant) {
  switch (octant & 7) {
  case 0:
    return {0, 1};
  case 1:
    return {1, 1};
  case 2:
    return {1, 0};
  case 3:
    return {1, -1};
  case 4:
    return {0, -1};
  case 5:
    return {-1, -1};
  case 6:
    return {-1, 0};
  case 7:
    return {-1, 1};
  default:
    return {0, 0};
  }
}

inline Cell cellOf(const WorldCoord &w) {
  return Cell{static_cast<int>(std::floor(w.x / CELL_SIZE_METERS)),
              static_cast<int>(std::floor(w.y / CELL_SIZE_METERS))};
}

inline Dir dirOf(double theta) {
  double deg = std::fmod(theta * 180.0 / M_PI, 360.0);
  if (deg < 0)
    deg += 360;
  if (deg >= 315 || deg < 45)
    return Dir::East;
  if (deg < 135)
    return Dir::North;
  if (deg < 225)
    return Dir::West;
  return Dir::South;
}

inline WorldCoord cellRelative(const WorldCoord &w, Cell c) {
  return WorldCoord{w.x - c.x * CELL_SIZE_METERS, w.y - c.y * CELL_SIZE_METERS,
                    w.theta};
}

struct Goals {
  const Cell *cells;
  int count;
  int explorationWeight = 1;
  int turnPenalty = 0;
};

constexpr int TIEBREAK_OFF = 0;

inline constexpr Cell centerGoals[]{{7, 7}, {7, 8}, {8, 7}, {8, 8}};
inline constexpr Cell startGoal[]{{0, 0}};

inline constexpr Goals CENTER_GOALS{centerGoals, 4,
                                    /*explorationWeight=*/TIEBREAK_OFF,
                                    /*turnPenalty=*/TIEBREAK_OFF};
inline constexpr Goals START_GOALS{startGoal, 1,
                                   /*explorationWeight=*/TIEBREAK_OFF,
                                   /*turnPenalty=*/TIEBREAK_OFF};

inline bool atGoal(Cell c, const Goals &goal) {
  for (int i = 0; i < goal.count; ++i)
    if (goal.cells[i] == c)
      return true;
  return false;
}
