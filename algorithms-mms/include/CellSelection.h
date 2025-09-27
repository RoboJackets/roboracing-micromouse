#pragma once
#include <vector>

#include "Mouse.h"

namespace CellSelection {
std::vector<Path> selectAllPaths(MouseState& state, const Goals* goal);

}  // namespace