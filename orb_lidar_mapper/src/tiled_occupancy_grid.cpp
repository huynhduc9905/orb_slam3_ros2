#include "orb_lidar_mapper/tiled_occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace orb_lidar_mapper {
namespace {

struct CellIndex {
  std::int64_t x{};
  std::int64_t y{};
};

std::int64_t floorCell(double coordinate, double resolution) {
  const double quotient = std::floor(coordinate / resolution);
  if (!std::isfinite(quotient) || quotient < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
      quotient > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
    throw std::out_of_range("ray coordinate cannot be represented as a grid cell");
  }
  return static_cast<std::int64_t>(quotient);
}

std::int64_t floorDivide(std::int64_t value, std::int64_t divisor) {
  const std::int64_t quotient = value / divisor;
  const std::int64_t remainder = value % divisor;
  return quotient - (remainder < 0 ? 1 : 0);
}

}  // namespace

struct TiledOccupancyGrid::Tile {
  std::int64_t x{};
  std::int64_t y{};
  std::vector<GridCell> cells;
};

TiledOccupancyGrid::~TiledOccupancyGrid() = default;

int8_t GridSnapshot::cellAt(Point2 cell) const {
  const auto cell_x = static_cast<std::int64_t>(cell.x);
  const auto cell_y = static_cast<std::int64_t>(cell.y);
  constexpr std::int64_t kTileSize = 64;
  const auto tile_x = floorDivide(cell_x, kTileSize);
  const auto tile_y = floorDivide(cell_y, kTileSize);
  const auto tile = std::find_if(tiles.begin(), tiles.end(), [tile_x, tile_y](const auto& candidate) {
    return candidate.tile_x == tile_x && candidate.tile_y == tile_y;
  });
  if (tile == tiles.end()) {
    return -1;
  }
  const auto local_x = static_cast<std::size_t>(cell_x - tile_x * kTileSize);
  const auto local_y = static_cast<std::size_t>(cell_y - tile_y * kTileSize);
  const GridCell& value = tile->cells[local_y * kTileSize + local_x];
  if (!value.observed ||
      (value.log_odds > free_threshold && value.log_odds < occupied_threshold)) {
    return -1;
  }
  return value.log_odds >= occupied_threshold ? 100 : 0;
}

TiledOccupancyGrid::TiledOccupancyGrid(GridConfig config) : config_(config) {
  if (config_.resolution_m <= 0.0 || config_.tile_size != 64 || config_.usable_range_m <= 0.0 ||
      config_.min_log_odds > config_.max_log_odds) {
    throw std::invalid_argument("invalid tiled occupancy grid configuration");
  }
}

void TiledOccupancyGrid::insert(const std::vector<Ray2>& rays) {
  const auto findTile = [this](std::int64_t tile_x, std::int64_t tile_y) -> Tile& {
    const auto found = std::find_if(tiles_.begin(), tiles_.end(), [tile_x, tile_y](const Tile& tile) {
      return tile.x == tile_x && tile.y == tile_y;
    });
    if (found != tiles_.end()) {
      return *found;
    }
    tiles_.push_back({tile_x, tile_y, std::vector<GridCell>(4096)});
    return tiles_.back();
  };
  const auto update = [this, &findTile](CellIndex index, float delta) {
    const auto tile_x = floorDivide(index.x, 64);
    const auto tile_y = floorDivide(index.y, 64);
    Tile& tile = findTile(tile_x, tile_y);
    const auto local_x = static_cast<std::size_t>(index.x - tile_x * 64);
    const auto local_y = static_cast<std::size_t>(index.y - tile_y * 64);
    GridCell& cell = tile.cells[local_y * 64 + local_x];
    cell.log_odds = std::clamp(cell.log_odds + delta, config_.min_log_odds, config_.max_log_odds);
    cell.observed = true;
  };

  for (const Ray2& ray : rays) {
    if (!std::isfinite(ray.origin.x) || !std::isfinite(ray.origin.y) || std::isnan(ray.end.x) ||
        std::isnan(ray.end.y)) {
      continue;
    }
    double dx = ray.end.x - ray.origin.x;
    double dy = ray.end.y - ray.origin.y;
    if (!ray.has_hit && (!std::isfinite(dx) || !std::isfinite(dy))) {
      dx = std::isinf(dx) ? std::copysign(1.0, dx) : 0.0;
      dy = std::isinf(dy) ? std::copysign(1.0, dy) : 0.0;
    }
    const double length = std::hypot(dx, dy);
    if (!std::isfinite(length) || length == 0.0) {
      if (ray.has_hit && length == 0.0) {
        update({floorCell(ray.origin.x, config_.resolution_m), floorCell(ray.origin.y, config_.resolution_m)},
               config_.hit_log_odds);
      }
      continue;
    }
    const double used_length = ray.has_hit ? length : config_.usable_range_m;
    if (ray.has_hit && used_length > config_.usable_range_m) {
      continue;
    }
    const CellIndex start{floorCell(ray.origin.x, config_.resolution_m),
                          floorCell(ray.origin.y, config_.resolution_m)};
    const CellIndex end{floorCell(ray.origin.x + dx / length * used_length, config_.resolution_m),
                        floorCell(ray.origin.y + dy / length * used_length, config_.resolution_m)};
    std::int64_t x = start.x;
    std::int64_t y = start.y;
    const std::int64_t step_x = x < end.x ? 1 : -1;
    const std::int64_t step_y = y < end.y ? 1 : -1;
    const std::int64_t delta_x = std::llabs(end.x - x);
    const std::int64_t delta_y = std::llabs(end.y - y);
    std::int64_t error = delta_x - delta_y;
    while (true) {
      if (!ray.has_hit || x != end.x || y != end.y) {
        update({x, y}, config_.miss_log_odds);
      }
      if (x == end.x && y == end.y) {
        break;
      }
      const std::int64_t twice_error = 2 * error;
      if (twice_error > -delta_y) {
        error -= delta_y;
        x += step_x;
      }
      if (twice_error < delta_x) {
        error += delta_x;
        y += step_y;
      }
    }
    if (ray.has_hit) {
      update(end, config_.hit_log_odds);
    }
  }
}

GridSnapshot TiledOccupancyGrid::snapshot() const {
  GridSnapshot snapshot{config_.occupied_threshold, config_.free_threshold, {}};
  snapshot.tiles.reserve(tiles_.size());
  for (const Tile& tile : tiles_) {
    snapshot.tiles.push_back({tile.x, tile.y, tile.cells});
  }
  std::sort(snapshot.tiles.begin(), snapshot.tiles.end(), [](const auto& lhs, const auto& rhs) {
    return std::tie(lhs.tile_y, lhs.tile_x) < std::tie(rhs.tile_y, rhs.tile_x);
  });
  return snapshot;
}

}  // namespace orb_lidar_mapper
