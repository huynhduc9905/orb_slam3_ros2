#pragma once

#include <cstddef>
#include <cstdint>

#include "orb_lidar_mapper/tiled_occupancy_grid.hpp"

namespace orb_lidar_mapper {

// Count consecutive occupied (100) cells along +X from (start_cell_x, cell_y).
// Returns 0 if start is not occupied or out of bounds.
std::size_t occupiedRunLengthX(const GridSnapshot& snap,
                               std::int64_t start_cell_x,
                               std::int64_t cell_y);

}  // namespace orb_lidar_mapper
