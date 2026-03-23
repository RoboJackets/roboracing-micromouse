#include "FloodFillSolver.h"

namespace {

void turningPenalty(MouseState &state, const Goals *goal,
                    int destinationArray[N][N]) {
  for (int y = 0; y < N; ++y) {
    for (int x = 0; x < N; ++x) {
      destinationArray[y][x] = state.dists[y][x];
    }
  }

  if (!(state.walls[state.y][state.x] & TOP) && state.y + 1 < N) {
    destinationArray[state.y + 1][state.x] +=
        goal->turnPenalty * dirToDist(state.dir, TOP);
  }
  if (!(state.walls[state.y][state.x] & LEFT) && state.x - 1 >= 0) {
    destinationArray[state.y][state.x - 1] +=
        goal->turnPenalty * dirToDist(state.dir, LEFT);
  }
  if (!(state.walls[state.y][state.x] & DOWN) && state.y - 1 >= 0) {
    destinationArray[state.y - 1][state.x] +=
        goal->turnPenalty * dirToDist(state.dir, DOWN);
  }
  if (!(state.walls[state.y][state.x] & RIGHT) && state.x + 1 < N) {
    destinationArray[state.y][state.x + 1] +=
        goal->turnPenalty * dirToDist(state.dir, RIGHT);
  }
}

void floodFill(MouseState &state, const Goals *goal, bool fast = false) {
  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      state.dists[y][x] = INF;
    }
  }

  std::queue<GridCoord> queue{};
  for (int i = 0; i < goal->count; ++i) {
    const int *g = goal->cells[i];
    state.dists[g[0]][g[1]] = 0;
    queue.push({g[1], g[0]});
  }

  while (!queue.empty()) {
    const GridCoord c = queue.front();
    queue.pop();

    const unsigned char wall = state.walls[c.y][c.x];
    const int dist = state.dists[c.y][c.x];

    // left
    if (c.x - 1 >= 0 && !(wall & LEFT) &&
        (!fast || state.explored[c.y][c.x - 1]) &&
        dist + 1 < state.dists[c.y][c.x - 1]) {
      state.dists[c.y][c.x - 1] = dist + 1;
      queue.push({c.x - 1, c.y});
    }
    // right
    if (c.x + 1 < N && !(wall & RIGHT) &&
        (!fast || state.explored[c.y][c.x + 1]) &&
        dist + 1 < state.dists[c.y][c.x + 1]) {
      state.dists[c.y][c.x + 1] = dist + 1;
      queue.push({c.x + 1, c.y});
    }
    // down
    if (c.y - 1 >= 0 && !(wall & DOWN) &&
        (!fast || state.explored[c.y - 1][c.x]) &&
        dist + 1 < state.dists[c.y - 1][c.x]) {
      state.dists[c.y - 1][c.x] = dist + 1;
      queue.push({c.x, c.y - 1});
    }
    // up
    if (c.y + 1 < N && !(wall & TOP) &&
        (!fast || state.explored[c.y + 1][c.x]) &&
        dist + 1 < state.dists[c.y + 1][c.x]) {
      state.dists[c.y + 1][c.x] = dist + 1;
      queue.push({c.x, c.y + 1});
    }
  }
}

void applyTiebreaker(MouseState &state, const Goals *goal,
                     int destinationArray[N][N]) {
  turningPenalty(state, goal, destinationArray);
  for (int x = 0; x < N; ++x) {
    for (int y = 0; y < N; ++y) {
      if (!state.explored[y][x]) {
        destinationArray[y][x] -= goal->explorationWeight;
      }
    }
  }
}

unsigned char traverse(MouseState &state, const Goals *goal) {
  auto fillNeighborCosts = [&](int out[4], int destination[N][N]) {
    out[0] = out[1] = out[2] = out[3] = INF + 200;
    const unsigned char here = state.walls[state.y][state.x];
    if (!(here & TOP) && state.y + 1 < N)
      out[0] = destination[state.y + 1][state.x];
    if (!(here & LEFT) && state.x - 1 >= 0)
      out[1] = destination[state.y][state.x - 1];
    if (!(here & DOWN) && state.y - 1 >= 0)
      out[2] = destination[state.y - 1][state.x];
    if (!(here & RIGHT) && state.x + 1 < N)
      out[3] = destination[state.y][state.x + 1];
  };

  int bestDirArray[4];
  fillNeighborCosts(bestDirArray, state.dists);

  // Serial.print("TOP: "); Serial.print(bestDirArray[0]);
  // Serial.print(" LEFT: "); Serial.print(bestDirArray[1]);
  // Serial.print(" DOWN: "); Serial.print(bestDirArray[2]);
  // Serial.print(" RIGHT: "); Serial.println(bestDirArray[3]);

  int best = INF + 100;
  bool tie = false;
  int bestDirID = -1;

  for (int i = 0; i < 4; ++i) {
    if (bestDirArray[i] == best)
      tie = true;
    if (bestDirArray[i] < best) {
      best = bestDirArray[i];
      bestDirID = i;
      tie = false;
    }
  }

  if (tie) {
    int destinationArray[N][N];
    applyTiebreaker(state, goal, destinationArray);
    fillNeighborCosts(bestDirArray, destinationArray);
    best = INF + 100;
    bestDirID = -1;
    for (int i = 0; i < 4; ++i) {
      if (bestDirArray[i] < best) {
        best = bestDirArray[i];
        bestDirID = i;
      }
    }
  }

  unsigned char bestDir = 0;
  switch (bestDirID) {
  case 0:
    bestDir = TOP;
    break;
  case 1:
    bestDir = LEFT;
    break;
  case 2:
    bestDir = DOWN;
    break;
  case 3:
    bestDir = RIGHT;
    break;
  default:
    bestDir = state.dir;
    break;
  }
  return bestDir;
}
}; // namespace
static unsigned char stepInstr(unsigned char cur, unsigned char tgt, bool fast) {
  unsigned char localTgt = tgt;
  if (cur == LEFT)
    localTgt = RCIRC4(tgt);
  if (cur == RIGHT)
    localTgt = LCIRC4(tgt);
  if (cur == DOWN)
    localTgt = LCIRC4(LCIRC4(tgt));
  if (fast) {
    switch (localTgt) {
    case TOP:
      return FWD0 + 1;
    case LEFT:
      return ST90L;
    case RIGHT:
      return ST90R;
    case DOWN:
      return IPT180;
    default:
      return FWD0;
    }
  }
  switch (localTgt) {
  case TOP:
    return EX_FWD0 + 1;
  case LEFT:
    return EX_ST90L;
  case RIGHT:
    return EX_ST90R;
  case DOWN:
    return IPT180;
  default:
    return EX_FWD0;
  }
}

Action *FloodFillSolver::run(MouseState &state, const Goals *goal) {
  if (atGoal(state, goal)) {
    cmd.load({STOP});
    return &cmd;
  }
  floodFill(state, goal, fast);
  unsigned char dir = traverse(state, goal);
  unsigned char c = stepInstr(state.dir, dir, fast);
  cmd.load({c});
  return &cmd;
}

bool FloodFillSolver::end(MouseState &state, const Goals *goal) {
  // std::cerr << std::to_string() << std::endl;
  return atGoal(state, goal);
}
