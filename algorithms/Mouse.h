#define RCIRC4(x) (((x) & 1) ? (0b1000 | (x) >> 1) : (x) >> 1)
#define LCIRC4(x) (((x) & 0b1000) ? (0b0001 | ((x) & ~0b1000) << 1) : (x) << 1)

constexpr unsigned char TOP{0b1000};
constexpr unsigned char RIGHT{0b0100};
constexpr unsigned char DOWN{0b0010};
constexpr unsigned char LEFT{0b0001};

struct Coord {
  int x;
  int y;
};
struct MouseState {
  int x = 0;
  int y = 0;
  unsigned char dir = TOP;  // 1000: top, 0100: right, 0010: down, 0001: left
};