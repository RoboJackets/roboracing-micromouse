#pragma once
#include <string>
#include <vector>
enum ActionState {
  START,
  ORTHO_F,
  ORTHO_L,
  ORTHO_R,
  DIAG_LR,
  DIAG_RL,
  E_DIAG_LR,
  E_DIAG_RL,
  END
};

struct State {
  ActionState action = START;
  unsigned char x = 0;
  unsigned char y = 0;
};

std::vector<unsigned char> parse(std::string s);
std::string commandString(const std::vector<unsigned char>& commands);