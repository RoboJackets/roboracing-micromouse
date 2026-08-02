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
void search_all(const MouseState &state, const Goals *goal);
std::vector<unsigned char> getCmds();
} // namespace CellSelection