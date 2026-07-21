#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/tiled_occupancy_grid.hpp"
#include "orb_lidar_mapper/wall_thickness.hpp"

namespace orb_lidar_mapper {
namespace {

GridSnapshot makeSnapshot(std::uint32_t width, std::uint32_t height,
                          std::int64_t origin_cell_x, std::int64_t origin_cell_y,
                          std::vector<std::int8_t> cells) {
  GridSnapshot snap;
  snap.resolution_m = 1.0;
  snap.origin_x = static_cast<double>(origin_cell_x);
  snap.origin_y = static_cast<double>(origin_cell_y);
  snap.origin_cell_x = origin_cell_x;
  snap.origin_cell_y = origin_cell_y;
  snap.width = width;
  snap.height = height;
  snap.cells = std::move(cells);
  return snap;
}

TEST(WallThickness, CountsOccupiedRunAlongX) {
  // Row y=0: free, occupied, occupied, occupied, free  (cells at x=0..4)
  // index = y * width + x
  auto snap = makeSnapshot(5, 1, 0, 0, {0, 100, 100, 100, 0});
  EXPECT_EQ(occupiedRunLengthX(snap, 1, 0), 3U);
  EXPECT_EQ(occupiedRunLengthX(snap, 2, 0), 2U);
  EXPECT_EQ(occupiedRunLengthX(snap, 3, 0), 1U);
}

TEST(WallThickness, ReturnsZeroWhenStartNotOccupied) {
  auto snap = makeSnapshot(5, 1, 0, 0, {0, 100, 100, 100, 0});
  EXPECT_EQ(occupiedRunLengthX(snap, 0, 0), 0U);
  EXPECT_EQ(occupiedRunLengthX(snap, 4, 0), 0U);
}

TEST(WallThickness, ReturnsZeroWhenOutOfBounds) {
  auto snap = makeSnapshot(3, 1, 0, 0, {100, 100, 100});
  EXPECT_EQ(occupiedRunLengthX(snap, -1, 0), 0U);
  EXPECT_EQ(occupiedRunLengthX(snap, 0, 1), 0U);
  EXPECT_EQ(occupiedRunLengthX(snap, 10, 0), 0U);
}

}  // namespace
}  // namespace orb_lidar_mapper
