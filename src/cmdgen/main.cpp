#include <iostream>
#include <string>
#include <vector>

#include "planner/CommandGenerator.h"

bool toMove(char c, Move &m) {
  switch (c) {
  case 'F': m = Move::Forward; return true;
  case 'L': m = Move::Left;    return true;
  case 'R': m = Move::Right;   return true;
  case 'B': m = Move::Back;    return true;
  case 'S': m = Move::Stop;    return true;
  }
  return false;
}

int main(int argc, char *argv[]) {
  std::vector<Move> moves;
  char c;
  while(std::cin >> c) {
    Move m;
    if (!toMove(c, m)) {
      std::cerr << "unknown move '" << c << "'\n";
      return 1;
    }
    moves.push_back(m);
  }

  CommandGenerator gen;
  for (const Command &c : gen.exec(moves)) {
    const unsigned char* cmd_bytes = reinterpret_cast<const unsigned char*>(&c);
    for (size_t i = 0; i < sizeof(c); ++i) std::cout << cmd_bytes[i];
  }

  return 0;
}
