#pragma once
#include <vector>

#include "CommandGenerator.h"
#include "Mouse.h"
#include <queue>
#include <sstream>
#include <stack>
#include <string>
#include <unordered_set>


namespace CellSelection {
Path pathBFS(MouseState &state, const Goals *goal);
Path weightedAStar(MouseState &state, const Goals *goal);
void search_all(const MouseState &state);
std::vector<unsigned char> getCmds();
} // namespace CellSelection