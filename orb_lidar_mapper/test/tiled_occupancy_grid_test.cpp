#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/tiled_occupancy_grid.hpp"

namespace orb_lidar_mapper {
namespace {

Ray2 ray(double end_x, bool has_hit = true) {
  return {{0.0, 0.0}, {end_x, 0.0}, has_hit};
}

TEST(TiledOccupancyGrid, HasSpecifiedDefaultConfiguration) {
  const GridConfig config;
  EXPECT_DOUBLE_EQ(config.resolution_m, 0.05);
  EXPECT_EQ(config.tile_size, 64);
  EXPECT_FLOAT_EQ(config.hit_log_odds, 0.85F);
  EXPECT_FLOAT_EQ(config.miss_log_odds, -0.40F);
  EXPECT_FLOAT_EQ(config.min_log_odds, -4.0F);
  EXPECT_FLOAT_EQ(config.max_log_odds, 4.0F);
  EXPECT_FLOAT_EQ(config.occupied_threshold, 0.619F);
  EXPECT_FLOAT_EQ(config.free_threshold, -0.619F);
  EXPECT_DOUBLE_EQ(config.usable_range_m, 12.0);
}

TEST(TiledOccupancyGrid, MarksFreeTraversalAndFiniteHitEndpoint) {
  GridConfig config;
  config.resolution_m = 1.0;
  TiledOccupancyGrid grid(config);
  grid.insert({ray(2.0), ray(2.0)});

  const GridSnapshot snapshot = grid.snapshot();
  EXPECT_EQ(snapshot.cellAt(0, 0), 0);
  EXPECT_EQ(snapshot.cellAt(1, 0), 0);
  EXPECT_EQ(snapshot.cellAt(2, 0), 100);
}

TEST(TiledOccupancyGrid, ProducesTaskFourCompatibleDenseSnapshotMetadataAndCells) {
  GridConfig config;
  config.resolution_m = 1.0;
  TiledOccupancyGrid grid(config);
  grid.insert({{{-1.0, 0.0}, {1.0, 0.0}, true},
               {{-1.0, 0.0}, {1.0, 0.0}, true},
               {{65.0, 64.0}, {66.0, 64.0}, true}});
  grid.insert({{{65.0, 64.0}, {66.0, 64.0}, true}});

  const GridSnapshot snapshot = grid.snapshot();
  EXPECT_DOUBLE_EQ(snapshot.resolution_m, 1.0);
  EXPECT_DOUBLE_EQ(snapshot.origin_x, -64.0);
  EXPECT_DOUBLE_EQ(snapshot.origin_y, 0.0);
  EXPECT_EQ(snapshot.width, 192U);
  EXPECT_EQ(snapshot.height, 128U);
  ASSERT_EQ(snapshot.cells.size(), 192U * 128U);
  EXPECT_EQ(snapshot.cellAt(-1, 0), 0);
  EXPECT_EQ(snapshot.cellAt(0, 0), 0);
  EXPECT_EQ(snapshot.cellAt(1, 0), 100);
  EXPECT_EQ(snapshot.cellAt(65, 64), 0);
  EXPECT_EQ(snapshot.cellAt(66, 64), 100);
  EXPECT_EQ(snapshot.cellAt(-64, 127), -1);
}

TEST(TiledOccupancyGrid, InfinityClearsWithoutOccupiedEndpointAndInvalidRangesAreSkipped) {
  GridConfig config;
  config.resolution_m = 1.0;
  config.usable_range_m = 3.0;
  TiledOccupancyGrid grid(config);
  grid.insert({ray(std::numeric_limits<double>::infinity(), false),
               ray(std::numeric_limits<double>::infinity(), false),
               ray(std::numeric_limits<double>::quiet_NaN())});

  const GridSnapshot snapshot = grid.snapshot();
  EXPECT_EQ(snapshot.cellAt(0, 0), 0);
  EXPECT_EQ(snapshot.cellAt(1, 0), 0);
  EXPECT_EQ(snapshot.cellAt(2, 0), 0);
  EXPECT_EQ(snapshot.cellAt(3, 0), 0);
  EXPECT_EQ(snapshot.cellAt(4, 0), -1);
}

TEST(TiledOccupancyGrid, FiniteRayBeyondUsableRangeClearsWithoutOccupiedEndpoint) {
  GridConfig config;
  config.resolution_m = 1.0;
  config.usable_range_m = 3.0;
  TiledOccupancyGrid grid(config);
  grid.insert({ray(10.0), ray(10.0)});

  const GridSnapshot snapshot = grid.snapshot();
  EXPECT_EQ(snapshot.cellAt(0, 0), 0);
  EXPECT_EQ(snapshot.cellAt(1, 0), 0);
  EXPECT_EQ(snapshot.cellAt(2, 0), 0);
  EXPECT_EQ(snapshot.cellAt(3, 0), 0);
  EXPECT_EQ(snapshot.cellAt(4, 0), -1);
}

TEST(TiledOccupancyGrid, UsesSignedFloorCellsAndNegativeTileKeys) {
  GridConfig config;
  config.resolution_m = 1.0;
  TiledOccupancyGrid grid(config);
  grid.insert({{{-64.1, -0.1}, {-65.1, -0.1}, true},
               {{-64.1, -0.1}, {-65.1, -0.1}, true}});

  const GridSnapshot snapshot = grid.snapshot();
  EXPECT_EQ(snapshot.cellAt(-65, -1), 0);
  EXPECT_EQ(snapshot.cellAt(-66, -1), 100);
}

TEST(TiledOccupancyGrid, SaturatesLogOddsAndKeepsAmbiguousObservedCellsUnknown) {
  GridConfig config;
  config.resolution_m = 1.0;
  config.usable_range_m = 100.0;
  TiledOccupancyGrid grid(config);
  for (int index = 0; index < 20; ++index) {
    grid.insert({ray(0.0)});
  }
  const auto saturated = grid.snapshot();
  EXPECT_EQ(saturated.cellAt(0, 0), 100);

  TiledOccupancyGrid ambiguous(config);
  ambiguous.insert({{{0.0, 0.0}, {1.0, 0.0}, true}});
  EXPECT_EQ(ambiguous.snapshot().cellAt(0, 0), -1);
}

TEST(TiledOccupancyGrid, ClassifiesSnapshotUsingConfiguredThresholds) {
  GridConfig config;
  config.resolution_m = 1.0;
  config.free_threshold = -0.1F;
  config.occupied_threshold = 0.1F;
  TiledOccupancyGrid grid(config);
  grid.insert({{{0.0, 0.0}, {1.0, 0.0}, true}});

  const GridSnapshot snapshot = grid.snapshot();
  EXPECT_EQ(snapshot.cellAt(0, 0), 0);
  EXPECT_EQ(snapshot.cellAt(1, 0), 100);
}

TEST(TiledOccupancyGrid, SerializesDenseBoundsAndCellsIndependentOfInsertionOrder) {
  GridConfig config;
  config.resolution_m = 1.0;
  config.usable_range_m = 100.0;
  const std::vector<Ray2> rays = {ray(65.0), {{-1.0, -65.0}, {-2.0, -65.0}, true}};
  TiledOccupancyGrid first(config);
  TiledOccupancyGrid second(config);
  first.insert(rays);
  second.insert({rays[1], rays[0]});

  EXPECT_EQ(first.snapshot(), second.snapshot());
}

TEST(TiledOccupancyGrid, SkipsExtremeFiniteRaysWithoutThrowing) {
  TiledOccupancyGrid grid;
  EXPECT_NO_THROW(grid.insert({{{1.0e300, 1.0e300}, {1.0e300, 1.0e300}, true},
                               {{-1.0e300, 0.0}, {-1.0e300, 1.0}, false}}));
  EXPECT_TRUE(grid.snapshot().cells.empty());
}

TEST(TiledOccupancyGrid, RejectsInvalidConfigurations) {
  const auto invalid = [](GridConfig config) {
    EXPECT_THROW(TiledOccupancyGrid grid(config), std::invalid_argument);
  };
  GridConfig config;
  config.resolution_m = std::numeric_limits<double>::infinity();
  invalid(config);
  config = {};
  config.usable_range_m = std::numeric_limits<double>::quiet_NaN();
  invalid(config);
  config = {};
  config.tile_size = 63;
  invalid(config);
  config = {};
  config.hit_log_odds = std::numeric_limits<float>::infinity();
  invalid(config);
  config = {};
  config.min_log_odds = 1.0F;
  config.max_log_odds = -1.0F;
  invalid(config);
  config = {};
  config.miss_log_odds = 0.0F;
  invalid(config);
  config = {};
  config.free_threshold = config.occupied_threshold;
  invalid(config);
  config = {};
  config.occupied_threshold = config.max_log_odds + 1.0F;
  invalid(config);
}

TEST(TiledOccupancyGrid, UsesExactThresholdBoundaries) {
  GridConfig config;
  config.resolution_m = 1.0;
  config.hit_log_odds = 1.0F;
  config.miss_log_odds = -1.0F;
  config.min_log_odds = -1.0F;
  config.max_log_odds = 1.0F;
  config.free_threshold = -1.0F;
  config.occupied_threshold = 1.0F;
  TiledOccupancyGrid grid(config);
  grid.insert({{{0.0, 0.0}, {1.0, 0.0}, true}});
  EXPECT_EQ(grid.snapshot().cellAt(0, 0), 0);
  EXPECT_EQ(grid.snapshot().cellAt(1, 0), 100);
}

}  // namespace
}  // namespace orb_lidar_mapper
