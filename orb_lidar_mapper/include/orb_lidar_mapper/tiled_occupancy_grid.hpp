#pragma once

#include <cstdint>
#include <vector>

#include "orb_lidar_mapper/scan_deskewer.hpp"

namespace orb_lidar_mapper {

struct GridConfig {
  double resolution_m{0.05};
  int tile_size{64};
  float hit_log_odds{0.85F};
  float miss_log_odds{-0.40F};
  float min_log_odds{-4.0F};
  float max_log_odds{4.0F};
  float occupied_threshold{0.619F};
  float free_threshold{-0.619F};
  double usable_range_m{12.0};
};

struct GridSnapshot {
  double resolution_m{};
  double origin_x{};
  double origin_y{};
  std::uint32_t width{};
  std::uint32_t height{};
  std::vector<std::int8_t> cells;

  std::int8_t cellAt(std::int64_t cell_x, std::int64_t cell_y) const;
  bool operator==(const GridSnapshot& rhs) const {
    return resolution_m == rhs.resolution_m && origin_x == rhs.origin_x && origin_y == rhs.origin_y &&
           width == rhs.width && height == rhs.height && cells == rhs.cells;
  }
};

class TiledOccupancyGrid {
 public:
  explicit TiledOccupancyGrid(GridConfig config = {});
  ~TiledOccupancyGrid();

  void insert(const std::vector<Ray2>& rays);
  GridSnapshot snapshot() const;

 private:
  struct Tile;

  GridConfig config_;
  std::vector<Tile> tiles_;
};

}  // namespace orb_lidar_mapper
