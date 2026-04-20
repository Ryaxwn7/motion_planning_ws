#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "incremental_esdf_2d.h"

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
    const double dist = std::sqrt(dx * dx + dy * dy) * resolution;
    if (dist < best) {
      best = dist;
    }
  }
  return best;
}

void expectMatchesBruteforce(
    const graph_planner::IncrementalEsdf2D& esdf,
    const std::vector<uint8_t>& occ,
    const int width,
    const int height,
    const double resolution) {
  const std::vector<double>& dist_map = esdf.getDistanceMap();
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

}  // namespace

TEST(IncrementalEsdf2D, InitializesFromOccupancy) {
  graph_planner::IncrementalEsdf2D esdf;
  const int width = 6;
  const int height = 5;
  const double resolution = 0.2;
  std::vector<uint8_t> occ(static_cast<std::size_t>(width * height), 0);
  occ[1] = 1;
  occ[width * 2 + 4] = 1;
  occ[width * 4 + 0] = 1;

  esdf.reset(width, height, resolution);
  esdf.initializeFromOccupancy(occ);

  ASSERT_TRUE(esdf.initialized());
  expectMatchesBruteforce(esdf, occ, width, height, resolution);
}

TEST(IncrementalEsdf2D, IncrementalInsertDeleteMatchesBruteforce) {
  graph_planner::IncrementalEsdf2D esdf;
  const int width = 7;
  const int height = 7;
  const double resolution = 0.1;
  std::vector<uint8_t> occ(static_cast<std::size_t>(width * height), 0);
  occ[width * 3 + 3] = 1;

  esdf.reset(width, height, resolution);
  esdf.initializeFromOccupancy(occ);
  expectMatchesBruteforce(esdf, occ, width, height, resolution);

  occ[width * 1 + 1] = 1;
  occ[width * 5 + 5] = 1;
  esdf.updateFromDiff({width * 1 + 1, width * 5 + 5}, {});
  expectMatchesBruteforce(esdf, occ, width, height, resolution);

  occ[width * 3 + 3] = 0;
  esdf.updateFromDiff({}, {width * 3 + 3});
  expectMatchesBruteforce(esdf, occ, width, height, resolution);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
