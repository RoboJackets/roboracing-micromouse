#pragma once
#include <array>

#include "Tuning.h"
#include "Types.h"

struct SensorMount {
  double x;
  double y;
  double theta;
};

// FL, FR, DL, DR
inline constexpr SensorMount IR_MOUNTS[4]{{-0.0473, 0.013, M_PI / 2},
                                          {0.0473, 0.013, M_PI / 2},
                                          {-0.021, 0.038, M_PI},
                                          {0.021, 0.038, 0}};

struct DistanceSensors {
  std::array<Translation2d, 4> readings{};
  std::array<Translation2d, 4> averages{};

  Translation2d buffer[4][IR_AVG_WINDOW]{};
  int bufIndex[4]{};
  int bufCount[4]{};

  void update(const std::array<double, 4> &metres);

  const std::array<Translation2d, 4> &state() const { return readings; }
  const std::array<Translation2d, 4> &averageState() const { return averages; }
};
