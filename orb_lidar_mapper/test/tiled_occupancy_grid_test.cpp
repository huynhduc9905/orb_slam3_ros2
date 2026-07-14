#include <cmath>
#include <limits>
#include <tuple>
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
  EXPECT_EQ(snapshot.cellAt({0, 0}), 0);
  EXPECT_EQ(snapshot.cellAt({1, 0}), 0);
  EXPECT_EQ(snapshot.cellAt({2, 0}), 100);
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
  EXPECT_EQ(snapshot.cellAt({0, 0}), 0);
  EXPECT_EQ(snapshot.cellAt({1, 0}), 0);
  EXPECT_EQ(snapshot.cellAt({2, 0}), 0);
  EXPECT_EQ(snapshot.cellAt({3, 0}), 0);
  EXPECT_EQ(snapshot.cellAt({4, 0}), -1);
}

TEST(TiledOccupancyGrid, UsesSignedFloorCellsAndNegativeTileKeys) {
  GridConfig config;
  config.resolution_m = 1.0;
  TiledOccupancyGrid grid(config);
  grid.insert({{{-64.1, -0.1}, {-65.1, -0.1}, true},
               {{-64.1, -0.1}, {-65.1, -0.1}, true}});

  const GridSnapshot snapshot = grid.snapshot();
  EXPECT_EQ(snapshot.cellAt({-65, -1}), 0);
  EXPECT_EQ(snapshot.cellAt({-66, -1}), 100);
  ASSERT_EQ(snapshot.tiles.size(), 1U);
  EXPECT_EQ(snapshot.tiles[0].tile_x, -2);
  EXPECT_EQ(snapshot.tiles[0].tile_y, -1);
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
  ASSERT_EQ(saturated.tiles.size(), 1U);
  EXPECT_FLOAT_EQ(saturated.tiles[0].cells[0].log_odds, config.max_log_odds);
  EXPECT_EQ(saturated.cellAt({0, 0}), 100);

  TiledOccupancyGrid ambiguous(config);
  ambiguous.insert({{{0.0, 0.0}, {1.0, 0.0}, true}});
  EXPECT_EQ(ambiguous.snapshot().cellAt({0, 0}), -1);
}

TEST(TiledOccupancyGrid, ClassifiesSnapshotUsingConfiguredThresholds) {
  GridConfig config;
  config.resolution_m = 1.0;
  config.free_threshold = -0.1F;
  config.occupied_threshold = 0.1F;
  TiledOccupancyGrid grid(config);
  grid.insert({{{0.0, 0.0}, {1.0, 0.0}, true}});

  const GridSnapshot snapshot = grid.snapshot();
  EXPECT_EQ(snapshot.cellAt({0, 0}), 0);
  EXPECT_EQ(snapshot.cellAt({1, 0}), 100);
}

TEST(TiledOccupancyGrid, SerializesSortedTilesAndCellsIndependentOfInsertionOrder) {
  GridConfig config;
  config.resolution_m = 1.0;
  config.usable_range_m = 100.0;
  const std::vector<Ray2> rays = {ray(65.0), {{-1.0, -65.0}, {-2.0, -65.0}, true}};
  TiledOccupancyGrid first(config);
  TiledOccupancyGrid second(config);
  first.insert(rays);
  second.insert({rays[1], rays[0]});

  EXPECT_EQ(first.snapshot(), second.snapshot());
  const auto snapshot = first.snapshot();
  ASSERT_GE(snapshot.tiles.size(), 2U);
  for (std::size_t index = 1; index < snapshot.tiles.size(); ++index) {
    EXPECT_LE(std::tie(snapshot.tiles[index - 1].tile_y, snapshot.tiles[index - 1].tile_x),
              std::tie(snapshot.tiles[index].tile_y, snapshot.tiles[index].tile_x));
  }
}

}  // namespace
}  // namespace orb_lidar_mapper
