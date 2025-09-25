#include "Helpers.h"
constexpr int centerGoals[][2] = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
constexpr int startGoal[][2] = {{0, 0}};

enum class GoalType { Center, Start };

struct Goals {
  const int (*cells)[2];
  int count;
  int explorationWeight = 1;
  int turnPenalty = 0;
};

constexpr Goals CENTER_GOALS{centerGoals, 4, 0, 1};
constexpr Goals START_GOALS{startGoal, 1, 3, 0};

constexpr int N = 16;
constexpr int INF = 300;
struct Coord {
  int x = 0;
  int y = 0;
};
struct MouseState {
  int x = 0;
  int y = 0;
  unsigned char dir = TOP;  // 1000: top, 0100: right, 0010: down, 0001: left
  Goals currentGoals = CENTER_GOALS;
  GoalType type = GoalType::Center;

  int dists[N][N];
  bool explored[N][N];
  unsigned char walls[N][N];  // 1000: top, 0100: right, 0010: down, 0001: left
  void setGoalType(GoalType t) {
    currentGoals = (t == GoalType::Center) ? CENTER_GOALS : START_GOALS;
  }
  void toggleGoalType() {
    (type == GoalType::Center) ? (type = GoalType::Start)
                               : (type = GoalType::Center);
    setGoalType(type);
  }
};

bool atGoal(MouseState& state) {
  for (int i = 0; i < state.currentGoals.count; ++i) {
    const int* g = state.currentGoals.cells[i];
    if (g[0] == state.y && g[1] == state.x) return true;
  }
  return false;
}