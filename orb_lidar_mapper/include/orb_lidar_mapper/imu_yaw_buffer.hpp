#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace orb_lidar_mapper {

struct TimedYawRate {
  std::int64_t stamp_ns{};
  double omega_rad_s{};
};

class ImuYawBuffer {
 public:
  explicit ImuYawBuffer(std::int64_t retention_ns,
                        std::int64_t max_gap_ns = 20'000'000LL);

  // Returns false if non-finite omega or non-monotonic stamp (strictly decreasing).
  // Equal stamp: average omega into last sample (duplicate stamp policy).
  bool push(TimedYawRate sample);

  // True if rates cover [start,end] with no gap > max_gap_ns.
  bool covers(std::int64_t start_ns, std::int64_t end_ns) const;

  // Trapezoidal integrate ω from start to target; nullopt if !covers(start,target).
  std::optional<double> integratedYaw(std::int64_t start_ns,
                                      std::int64_t target_ns) const;

  // Stamp of the newest sample, if any (for mapper wait-vs-fallback decisions).
  std::optional<std::int64_t> newestStamp() const noexcept;

  std::size_t size() const noexcept;

 private:
  std::deque<TimedYawRate> samples_;
  std::int64_t retention_ns_;
  std::int64_t max_gap_ns_;
  std::size_t last_duplicate_count_{0};
};

}  // namespace orb_lidar_mapper
