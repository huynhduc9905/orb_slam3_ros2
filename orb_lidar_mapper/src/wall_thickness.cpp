#include "orb_lidar_mapper/wall_thickness.hpp"

#include <limits>

namespace orb_lidar_mapper {

std::size_t occupiedRunLengthX(const GridSnapshot& snap,
                               std::int64_t start_cell_x,
                               std::int64_t cell_y) {
  if (snap.cellAt(start_cell_x, cell_y) != 100) {
    return 0;
  }
  std::size_t count = 0;
  for (std::int64_t x = start_cell_x;; ++x) {
    if (snap.cellAt(x, cell_y) != 100) {
      break;
    }
    ++count;
    // Guard against overflow if origin is near int64 max (cellAt returns -1 OOB).
    if (x == std::numeric_limits<std::int64_t>::max()) {
      break;
    }
  }
  return count;
}

}  // namespace orb_lidar_mapper
