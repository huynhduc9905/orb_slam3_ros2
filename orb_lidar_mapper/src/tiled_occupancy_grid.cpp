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

constexpr std::int64_t kTileSize = 64;
constexpr std::size_t kCellsPerTile = kTileSize * kTileSize;

struct CellIndex { std::int64_t x{}; std::int64_t y{}; };
struct GridCell { float log_odds{}; bool observed{}; };

bool finite(float value) { return std::isfinite(value); }

std::optional<std::int64_t> floorCell(double coordinate, double resolution) {
  const long double quotient = std::floor(static_cast<long double>(coordinate) /
                                          static_cast<long double>(resolution));
  // Keep enough room for the checked Bresenham deltas and tile calculations.
  constexpr auto kLimit = std::numeric_limits<std::int64_t>::max() / 4;
  if (!std::isfinite(quotient) || quotient <= -static_cast<long double>(kLimit) ||
      quotient >= static_cast<long double>(kLimit)) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(quotient);
}

std::int64_t floorDivide(std::int64_t value) {
  const std::int64_t quotient = value / kTileSize;
  return quotient - (value < 0 && value % kTileSize != 0 ? 1 : 0);
}

std::size_t localCoordinate(std::int64_t value) {
  const std::int64_t remainder = value % kTileSize;
  return static_cast<std::size_t>(remainder < 0 ? remainder + kTileSize : remainder);
}

bool validConfig(const GridConfig& config) {
  return std::isfinite(config.resolution_m) && config.resolution_m > 0.0 &&
         config.tile_size == kTileSize && std::isfinite(config.usable_range_m) &&
         config.usable_range_m > 0.0 && finite(config.hit_log_odds) && finite(config.miss_log_odds) &&
         finite(config.min_log_odds) && finite(config.max_log_odds) &&
         finite(config.free_threshold) && finite(config.occupied_threshold) &&
         config.min_log_odds <= 0.0F && 0.0F <= config.max_log_odds &&
         config.min_log_odds <= config.miss_log_odds && config.miss_log_odds < 0.0F &&
         0.0F < config.hit_log_odds && config.hit_log_odds <= config.max_log_odds &&
         config.min_log_odds <= config.free_threshold && config.free_threshold <= 0.0F &&
         0.0F <= config.occupied_threshold && config.occupied_threshold <= config.max_log_odds &&
         config.free_threshold < config.occupied_threshold;
}

}  // namespace

struct TiledOccupancyGrid::Tile {
  std::int64_t x{};
  std::int64_t y{};
  std::vector<GridCell> cells;
};

TiledOccupancyGrid::~TiledOccupancyGrid() = default;

std::int8_t GridSnapshot::cellAt(std::int64_t cell_x, std::int64_t cell_y) const {
  if (!std::isfinite(resolution_m) || resolution_m <= 0.0 || !std::isfinite(origin_x) ||
      !std::isfinite(origin_y) || width == 0 || height == 0 ||
      static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() / height) {
    return -1;
  }
  const std::size_t expected_cells = static_cast<std::size_t>(width) * height;
  if (cells.size() != expected_cells || cell_x < origin_cell_x || cell_y < origin_cell_y) return -1;
  const std::uint64_t x = static_cast<std::uint64_t>(cell_x) -
                          static_cast<std::uint64_t>(origin_cell_x);
  const std::uint64_t y = static_cast<std::uint64_t>(cell_y) -
                          static_cast<std::uint64_t>(origin_cell_y);
  if (x >= width || y >= height) return -1;
  return cells[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)];
}

TiledOccupancyGrid::TiledOccupancyGrid(GridConfig config) : config_(config) {
  if (!validConfig(config_)) throw std::invalid_argument("invalid tiled occupancy grid configuration");
}

void TiledOccupancyGrid::insert(const std::vector<Ray2>& rays) {
  const auto findTile = [this](std::int64_t tile_x, std::int64_t tile_y) -> Tile& {
    const auto found = std::find_if(tiles_.begin(), tiles_.end(), [=](const Tile& tile) {
      return tile.x == tile_x && tile.y == tile_y;
    });
    if (found != tiles_.end()) return *found;
    tiles_.push_back({tile_x, tile_y, std::vector<GridCell>(kCellsPerTile)});
    return tiles_.back();
  };
  const auto update = [this, &findTile](CellIndex index, float delta) {
    Tile& tile = findTile(floorDivide(index.x), floorDivide(index.y));
    GridCell& cell = tile.cells[localCoordinate(index.y) * kTileSize + localCoordinate(index.x)];
    cell.log_odds = std::clamp(cell.log_odds + delta, config_.min_log_odds, config_.max_log_odds);
    cell.observed = true;
  };

  for (const Ray2& ray : rays) {
    if (!std::isfinite(ray.origin.x) || !std::isfinite(ray.origin.y) || std::isnan(ray.end.x) ||
        std::isnan(ray.end.y)) continue;
    double dx = ray.end.x - ray.origin.x;
    double dy = ray.end.y - ray.origin.y;
    double length = std::hypot(dx, dy);
    const bool infinite_clear = !ray.has_hit && !std::isfinite(length);
    if (infinite_clear) {
      dx = std::isinf(dx) ? std::copysign(1.0, dx) : 0.0;
      dy = std::isinf(dy) ? std::copysign(1.0, dy) : 0.0;
      length = std::hypot(dx, dy);
    }
    if (!std::isfinite(length) || length == 0.0) {
      if (ray.has_hit && length == 0.0) {
        const auto x = floorCell(ray.origin.x, config_.resolution_m);
        const auto y = floorCell(ray.origin.y, config_.resolution_m);
        if (x && y) update({*x, *y}, config_.hit_log_odds);
      }
      continue;
    }
    const bool has_hit = ray.has_hit && length <= config_.usable_range_m;
    const double used_length = infinite_clear ? config_.usable_range_m :
                                              std::min(length, config_.usable_range_m);
    const auto start_x = floorCell(ray.origin.x, config_.resolution_m);
    const auto start_y = floorCell(ray.origin.y, config_.resolution_m);
    const auto end_x = floorCell(ray.origin.x + dx / length * used_length, config_.resolution_m);
    const auto end_y = floorCell(ray.origin.y + dy / length * used_length, config_.resolution_m);
    if (!start_x || !start_y || !end_x || !end_y) continue;
    const CellIndex start{*start_x, *start_y};
    const CellIndex end{*end_x, *end_y};
    const std::int64_t delta_x = std::llabs(end.x - start.x);
    const std::int64_t delta_y = std::llabs(end.y - start.y);
    if (delta_x > std::numeric_limits<std::int64_t>::max() / 4 ||
        delta_y > std::numeric_limits<std::int64_t>::max() / 4) continue;
    std::int64_t x = start.x, y = start.y;
    const std::int64_t step_x = x < end.x ? 1 : -1, step_y = y < end.y ? 1 : -1;
    std::int64_t error = delta_x - delta_y;
    while (true) {
      if (!has_hit || x != end.x || y != end.y) update({x, y}, config_.miss_log_odds);
      if (x == end.x && y == end.y) break;
      const std::int64_t twice_error = error * 2;
      if (twice_error > -delta_y) { error -= delta_y; x += step_x; }
      if (twice_error < delta_x) { error += delta_x; y += step_y; }
    }
    if (has_hit) update(end, config_.hit_log_odds);
  }
}

GridSnapshot TiledOccupancyGrid::snapshot() const {
  GridSnapshot result;
  result.resolution_m = config_.resolution_m;
  if (tiles_.empty()) return result;
  std::vector<const Tile*> ordered;
  ordered.reserve(tiles_.size());
  for (const Tile& tile : tiles_) ordered.push_back(&tile);
  std::sort(ordered.begin(), ordered.end(), [](const Tile* lhs, const Tile* rhs) {
    return std::tie(lhs->y, lhs->x) < std::tie(rhs->y, rhs->x);
  });
  const auto [min_x, max_x] = std::minmax_element(ordered.begin(), ordered.end(),
      [](const Tile* lhs, const Tile* rhs) { return lhs->x < rhs->x; });
  const auto [min_y, max_y] = std::minmax_element(ordered.begin(), ordered.end(),
      [](const Tile* lhs, const Tile* rhs) { return lhs->y < rhs->y; });
  const std::uint64_t tiles_wide = static_cast<std::uint64_t>((*max_x)->x - (*min_x)->x + 1);
  const std::uint64_t tiles_high = static_cast<std::uint64_t>((*max_y)->y - (*min_y)->y + 1);
  const std::uint64_t width = tiles_wide * kTileSize, height = tiles_high * kTileSize;
  if (width > std::numeric_limits<std::uint32_t>::max() || height > std::numeric_limits<std::uint32_t>::max() ||
      width > std::numeric_limits<std::size_t>::max() / height) return result;
  result.origin_x = static_cast<double>((*min_x)->x * kTileSize) * config_.resolution_m;
  result.origin_y = static_cast<double>((*min_y)->y * kTileSize) * config_.resolution_m;
  result.origin_cell_x = (*min_x)->x * kTileSize;
  result.origin_cell_y = (*min_y)->y * kTileSize;
  result.width = static_cast<std::uint32_t>(width);
  result.height = static_cast<std::uint32_t>(height);
  result.cells.assign(static_cast<std::size_t>(width * height), -1);
  for (const Tile* tile : ordered) {
    const std::size_t offset_x = static_cast<std::size_t>((tile->x - (*min_x)->x) * kTileSize);
    const std::size_t offset_y = static_cast<std::size_t>((tile->y - (*min_y)->y) * kTileSize);
    for (std::size_t y = 0; y < kTileSize; ++y) for (std::size_t x = 0; x < kTileSize; ++x) {
      const GridCell& cell = tile->cells[y * kTileSize + x];
      if (!cell.observed) continue;
      result.cells[(offset_y + y) * width + offset_x + x] =
          cell.log_odds >= config_.occupied_threshold ? 100 :
          (cell.log_odds <= config_.free_threshold ? 0 : -1);
    }
  }
  return result;
}

}  // namespace orb_lidar_mapper
