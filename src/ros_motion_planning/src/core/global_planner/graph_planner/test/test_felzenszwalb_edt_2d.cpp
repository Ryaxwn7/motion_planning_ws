#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "felzenszwalb_edt_2d.h"

namespace {

double bruteForceDistance(
    const std::vector<uint8_t>& occ,
    const int width,
    const int height,
    const double resolution,
    const int idx) {
  if (idx < 0 || idx >= static_cast<int>(occ.size())) {
    return std::numeric_limits<double>::infinity();
  }
  if (occ[static_cast<std::size_t>(idx)] != 0) {
    return 0.0;
  }
  const int x = idx % width;
  const int y = idx / width;
  double best = std::numeric_limits<double>::infinity();
  for (int j = 0; j < static_cast<int>(occ.size()); ++j) {
    if (occ[static_cast<std::size_t>(j)] == 0) {
      continue;
    }
    const int ox = j % width;
    const int oy = j / width;
    const double dx = static_cast<double>(x - ox);
    const double dy = static_cast<double>(y - oy);
    best = std::min(best, std::sqrt(dx * dx + dy * dy) * resolution);
  }
  return best;
}

TEST(FelzenszwalbEdt2D, MatchesBruteforceDistanceMap) {
  graph_planner::FelzenszwalbEdt2D edt;
  const int width = 8;
  const int height = 6;
  const double resolution = 0.15;
  std::vector<uint8_t> occ(static_cast<std::size_t>(width * height), 0);
  occ[1] = 1;
  occ[width * 2 + 4] = 1;
  occ[width * 5 + 6] = 1;

  edt.reset(width, height, resolution);
  edt.computeFromOccupancy(occ);

  ASSERT_TRUE(edt.initialized());
  const std::vector<double>& dist_map = edt.getDistanceMap();
  ASSERT_EQ(dist_map.size(), occ.size());
  for (int idx = 0; idx < static_cast<int>(occ.size()); ++idx) {
    const double expected = bruteForceDistance(occ, width, height, resolution, idx);
    const double actual = dist_map[static_cast<std::size_t>(idx)];
    if (!std::isfinite(expected)) {
      EXPECT_FALSE(std::isfinite(actual));
      continue;
    }
    EXPECT_NEAR(actual, expected, 1e-6) << "idx=" << idx;
  }
}

TEST(FelzenszwalbEdt2D, RecomputesAfterOccupancyChange) {
  graph_planner::FelzenszwalbEdt2D edt;
  const int width = 5;
  const int height = 5;
  const double resolution = 0.1;
  std::vector<uint8_t> occ(static_cast<std::size_t>(width * height), 0);
  occ[width * 2 + 2] = 1;

  edt.reset(width, height, resolution);
  edt.computeFromOccupancy(occ);
  ASSERT_NEAR(edt.getDistance(width * 2 + 1), 0.1, 1e-6);

  occ[width * 2 + 2] = 0;
  occ[width * 0 + 0] = 1;
  edt.computeFromOccupancy(occ);
  ASSERT_NEAR(edt.getDistance(width * 2 + 1), std::sqrt(5.0) * 0.1, 1e-6);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
