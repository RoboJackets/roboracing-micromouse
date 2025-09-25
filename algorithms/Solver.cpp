#include "Mouse.h"
class Solver {
 public:
  virtual void run(MouseState& state, const Goals* goal) { }
  virtual bool end(MouseState& state, const Goals* goal) { return true; }
  virtual void logType() { log("none"); }
};