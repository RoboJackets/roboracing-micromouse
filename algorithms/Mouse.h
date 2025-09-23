#define RCIRC4(x) (((x) & 1) ? (0b1000 | (x) >> 1) : (x) >> 1)
#define LCIRC4(x) (((x) & 0b1000) ? (0b0001 | ((x) & ~0b1000) << 1) : (x) << 1)

constexpr unsigned char TOP{0b1000};
constexpr unsigned char RIGHT{0b0100};
constexpr unsigned char DOWN{0b0010};
constexpr unsigned char LEFT{0b0001};
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
constexpr Goals START_GOALS{startGoal, 1, 1, 0};

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
  void setGoalType(GoalType t) {
    currentGoals = (t == GoalType::Center) ? CENTER_GOALS : START_GOALS;
  }
  void toggleGoalType() {
    (type == GoalType::Center) ? (type = GoalType::Start)
                               : (type = GoalType::Center);
    setGoalType(type);
  }
};