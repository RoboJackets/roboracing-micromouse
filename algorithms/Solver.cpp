#include "Mouse.h"
class Solver {
 public:
  virtual void run(MouseState& state) {}
  virtual bool end() { return true; }
  virtual void logType() { log("none"); }
};