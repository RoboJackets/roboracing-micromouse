#include "CommandGenerator.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Commands.h"

State transition(State currentState, char c,
                 std::vector<unsigned char>& commands) {
  unsigned char x = currentState.x;
  unsigned char y = currentState.y;

  if (c == 'S') {
    if (x > 0) {
      commands.push_back(FWD0 + x);
    }
    if (y > 0) {
      commands.push_back(DFWD0 + y);
    }
    commands.push_back(STOP);
    return {END, 0, 0};
  }

  switch (currentState.action) {
    case START:
      return {ORTHO_F};
    case ORTHO_F:
      if (c == 'F')
        return {ORTHO_F, x + 1};
      else if (c == 'L') {
        commands.push_back(FWD0 + x);
        return {ORTHO_L, 0};
      } else if (c == 'R') {
        commands.push_back(FWD0 + x);
        return {ORTHO_R, 0};
      }
      break;
    case ORTHO_L:
      if (c == 'F') {
        commands.push_back(ST90L);
        return {ORTHO_F, 1, 0};
      } else if (c == 'L') {
        commands.push_back(ST90L);
        return {ORTHO_L};
      } else if (c == 'R') {
        commands.push_back(ST45L);
        return {DIAG_LR, 0, 1};
      }
      break;
    case ORTHO_R:
      if (c == 'F') {
        commands.push_back(ST90R);
        return {ORTHO_F, 1, 0};
      } else if (c == 'L') {
        commands.push_back(ST45R);
        return {DIAG_RL, 0, 1};
      } else if (c == 'R') {
        commands.push_back(ST90R);
        return {ORTHO_R};
      }
      break;
    case DIAG_LR:
      if (c == 'F') {
        commands.push_back(DFWD0 + y);
        commands.push_back(ST45R);
        return {ORTHO_F, 1, 0};
      } else if (c == 'L') {
        return {E_DIAG_LR, 0, y};
      } else if (c == 'R') {
        commands.push_back(DFWD0 + y);
        commands.push_back(ST45R);
        return {ORTHO_R, 0, 0};
      }
      break;
    case DIAG_RL: {
      if (c == 'F') {
        commands.push_back(DFWD0 + y);
        commands.push_back(ST45L);
        return {ORTHO_F, 1, 0};
      } else if (c == 'L') {
        commands.push_back(DFWD0 + y);
        commands.push_back(ST45L);
        return {ORTHO_L, 0, 0};
      } else if (c == 'R') {
        return {E_DIAG_RL, 0, y};
      }
      break;
    }
    case E_DIAG_LR: {
      if (c == 'F') {
        commands.push_back(DFWD0 + y);
        commands.push_back(ST45L);
        return {ORTHO_F, 2, 0};
      } else if (c == 'L') {
        commands.push_back(DFWD0 + y);
        commands.push_back(ST45L);
        return {ORTHO_L, 0, 0};
      } else if (c == 'R') {
        return {DIAG_LR, 0, y + 1};
      }
      break;
    }
    case E_DIAG_RL: {
      if (c == 'F') {
        commands.push_back(DFWD0 + y);
        commands.push_back(ST45R);
        return {ORTHO_F, 2, 0};
      } else if (c == 'L') {
        return {DIAG_RL, 0, y + 1};
      } else if (c == 'R') {
        commands.push_back(DFWD0 + y);
        commands.push_back(ST45R);
        return {ORTHO_R, 0, 0};
      }
      break;
    }
  }
  return {END, 0, 0};
}

std::string commandString(const std::vector<unsigned char>& commands) {
  std::stringstream oss;

  auto append = [&](const std::string& s) {
    if (oss.tellp() > 0) oss << ", ";
    oss << s;
  };

  for (unsigned char c : commands) {
    unsigned char cmd = c & 0b11100000;
    unsigned char arg = c & 0b00011111;
    switch (cmd) {
      case FWD0:
        append("FWD" + std::to_string(arg));
        break;
      case DFWD0:
        append("DFWD" + std::to_string(arg));
        break;
      case ST0:
        if (c == ST0)
          append("ST0");
        else if (c == ST45L)
          append("ST45L");
        else if (c == ST90L)
          append("ST90L");
        else if (c == ST135L)
          append("ST135L");
        else if (c == ST45R)
          append("ST45R");
        else if (c == ST90R)
          append("ST90R");
        else if (c == ST135R)
          append("ST135R");
        else
          append("ST" + std::to_string(arg));
        break;
      case IPT0:
        if (c == IPT0)
          append("IPT0");
        else if (c == IPT45L)
          append("IPT45L");
        else if (c == IPT90L)
          append("IPT90L");
        else if (c == IPT135L)
          append("IPT135L");
        else if (c == IPT45R)
          append("IPT45R");
        else if (c == IPT90R)
          append("IPT90R");
        else if (c == IPT135R)
          append("IPT135R");
        else
          append("IPT" + std::to_string(arg));
        break;
      default:
        if (c == STOP)
          append("STOP");
        else
          append("UNK(" + std::to_string(c) + ")");
        break;
    }
  }
  return oss.str();
}

std::vector<unsigned char> parse(
    std::string s) {  // assumes no motion on 45 degree turns (diagonals) but on
                      // a left turn assumes one cell of motion as well.
  State current{};
  std::vector<unsigned char> commands{};
  for (int i = 0; i < s.length(); i++) {
    current = transition(current, s[i], commands);
  }
  std::cerr << commandString(commands) << std::endl;
  return commands;
}