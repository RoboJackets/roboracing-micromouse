#pragma once
#include <vector>

#include "Mouse.h"

namespace CellSelection {
Path selectPath(MouseState& state, const Goals* goal);

}  // namespace