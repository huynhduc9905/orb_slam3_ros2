#pragma once

#include "orb_slam3_wrapper/stereo_calib_pipeline.hpp"

namespace orb_slam3_wrapper {

// Writes under run.config.output_dir:
//   calibration.json, centers.csv, trajectory.csv, report.html
// Respects overwrite flag; creates directories. Never writes TF.
void writeStereoCalibrationReport(const StereoCalibRun& run);

}  // namespace orb_slam3_wrapper
