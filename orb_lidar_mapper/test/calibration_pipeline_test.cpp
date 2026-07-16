#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "orb_lidar_mapper/calibration_pipeline.hpp"

namespace orb_lidar_mapper {
namespace {

TEST(CalibrationPipeline, MapsScientificResultsToExactExitCodes) {
  EXPECT_EQ(resultExitCode(ResultClass::kConsistent), 0);
  EXPECT_EQ(resultExitCode(ResultClass::kLikelyOffsetError), 2);
  EXPECT_EQ(resultExitCode(ResultClass::kInconclusive), 3);
}

TEST(CalibrationPipeline, RejectsUnreadableBagAsOperationalFailure) {
  CalibrationConfig config;
  config.bag_path = "/path/that/does/not/exist";
  config.output_dir = std::filesystem::temp_directory_path() / "missing-bag-output";
  EXPECT_THROW(runCalibration(config), std::runtime_error);
}

}  // namespace
}  // namespace orb_lidar_mapper
