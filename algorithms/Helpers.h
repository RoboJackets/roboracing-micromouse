#include <iostream>

#define RCIRC4(x) (((x) & 1) ? (0b1000 | (x) >> 1) : (x) >> 1)
#define LCIRC4(x) (((x) & 0b1000) ? (0b0001 | ((x) & ~0b1000) << 1) : (x) << 1)
constexpr unsigned char TOP{0b1000};
constexpr unsigned char RIGHT{0b0100};
constexpr unsigned char DOWN{0b0010};
constexpr unsigned char LEFT{0b0001};
void log(const std::string& text) { std::cerr << text << '\n'; }

int dirToDist(unsigned char dir1, unsigned char dir2) {
  if (dir1 == dir2) return 0;

  unsigned char rot1 = dir1;
  unsigned char rot2 = dir2;

  if (dir1 == LEFT) {
    rot1 = RCIRC4(dir1);
    rot2 = RCIRC4(dir2);
  }
  if (dir1 == RIGHT) {
    rot1 = LCIRC4(dir1);
    rot2 = LCIRC4(dir2);
  }
  if (dir1 == DOWN) {
    rot1 = LCIRC4(LCIRC4(dir1));
    rot2 = LCIRC4(LCIRC4(dir2));
  }

  if (rot2 == LEFT || rot2 == RIGHT) return 1;
  return 2;
}

char convertDir(unsigned char dir) {
  switch (dir) {
    case TOP:
      return 'n';
    case LEFT:
      return 'w';
    case RIGHT:
      return 'e';
    case DOWN:
      return 's';
    default:
      return 'x';
  }
}