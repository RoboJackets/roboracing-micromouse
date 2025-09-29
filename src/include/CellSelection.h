#pragma once
#include <vector>

#include "Mouse.h"

namespace CellSelection {
Path pathBFS(MouseState& state, const Goals* goal);
Path weightedAStar(MouseState& state, const Goals* goal);
}  // namespace