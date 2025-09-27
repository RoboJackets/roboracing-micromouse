#pragma once
#include "Helpers.h"

constexpr int centerGoals[][2] = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
constexpr int startGoal[][2] = {{0, 0}};

struct Goals {
  const int (*cells)[2];
  int count;
  int explorationWeight = 1;
  int turnPenalty = 0;
};

constexpr Goals CENTER_GOALS{centerGoals, 4, 0, 1};
constexpr Goals START_GOALS{startGoal, 1, 3, 0};

struct MouseState {
  int x = 0;
  int y = 0;
  unsigned char dir = TOP;
  int dists[N][N]{};
  bool explored[N][N]{};
  unsigned char walls[N][N]{};  // bitmask
};

inline bool atGoal(const MouseState& state, const Goals* goal) {
  for (int i = 0; i < goal->count; ++i) {
    const int* g = goal->cells[i];
    if (g[0] == state.y && g[1] == state.x) return true;
  }
  return false;
}
