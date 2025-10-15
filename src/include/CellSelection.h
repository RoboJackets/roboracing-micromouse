#pragma once
#include <vector>

#include "Mouse.h"

namespace CellSelection {
Path pathBFS(MouseState& state, const Goals* goal);
Path weightedAStar(MouseState& state, const Goals* goal);
void search_all(const MouseState& state);
std::vector<unsigned char> getCmds();
}  // namespace