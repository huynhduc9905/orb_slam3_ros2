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

struct GridCell {
  float log_odds{};
  bool observed{};

  bool operator==(const GridCell& rhs) const {
    return log_odds == rhs.log_odds && observed == rhs.observed;
  }
};

struct GridTileSnapshot {
  std::int64_t tile_x{};
  std::int64_t tile_y{};
  std::vector<GridCell> cells;

  bool operator==(const GridTileSnapshot& rhs) const {
    return tile_x == rhs.tile_x && tile_y == rhs.tile_y && cells == rhs.cells;
  }
};

struct GridSnapshot {
  float occupied_threshold{0.619F};
  float free_threshold{-0.619F};
  std::vector<GridTileSnapshot> tiles;

  int8_t cellAt(Point2 cell) const;
  bool operator==(const GridSnapshot& rhs) const {
    return occupied_threshold == rhs.occupied_threshold && free_threshold == rhs.free_threshold &&
           tiles == rhs.tiles;
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
