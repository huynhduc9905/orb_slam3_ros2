#include "orb_slam3_wrapper/stereo_calib_analysis.hpp"
#include "orb_slam3_wrapper/stereo_calib_pipeline.hpp"
#include "orb_slam3_wrapper/stereo_calib_report.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

namespace orb_slam3_wrapper {
namespace {

void printHelp(const char* executable) {
  std::cout << "Usage: " << executable << " --bag PATH --output DIR [OPTIONS]\n"
            << "Options:\n"
            << "  --bag PATH\n"
            << "  --output DIR\n"
            << "  --overwrite\n"
            << "  --vocab PATH\n"
            << "  --settings PATH\n"
            << "  --help\n";
}

std::string defaultVocabularyPath() {
  return ament_index_cpp::get_package_share_directory("orb_slam3_vendor") +
         "/vocabulary/ORBvoc.txt";
}

std::string defaultSettingsPath() {
  return ament_index_cpp::get_package_share_directory("orb_slam3_wrapper") +
         "/config/tasterobot_stereo.yaml";
}

int parse(int argc, char** argv, StereoCalibConfig& config) {
  bool have_bag = false;
  bool have_output = false;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--help") {
      if (argc != 2) return 1;
      printHelp(argv[0]);
      return 0;
    }
    if (option == "--overwrite") {
      config.overwrite = true;
    } else if (option == "--bag" || option == "--output" || option == "--vocab" ||
               option == "--settings") {
      if (index + 1 >= argc) return 1;
      const std::string value = argv[++index];
      if (value.empty() || value.rfind("--", 0) == 0) return 1;
      if (option == "--bag") {
        config.bag_path = value;
        have_bag = true;
      } else if (option == "--output") {
        config.output_dir = value;
        have_output = true;
      } else if (option == "--vocab") {
        config.vocabulary_file = value;
      } else {
        config.settings_file = value;
      }
    } else {
      return 1;
    }
  }
  if (!have_bag || !have_output) return 1;
  if (config.vocabulary_file.empty()) {
    config.vocabulary_file = defaultVocabularyPath();
  }
  if (config.settings_file.empty()) {
    config.settings_file = defaultSettingsPath();
  }
  return -1;
}

const char* resultClassName(ResultClass result_class) {
  switch (result_class) {
    case ResultClass::kConsistent:
      return "CONSISTENT";
    case ResultClass::kLikelyOffsetError:
      return "LIKELY_OFFSET_ERROR";
    case ResultClass::kInconclusive:
      return "INCONCLUSIVE";
  }
  return "INCONCLUSIVE";
}

}  // namespace
}  // namespace orb_slam3_wrapper

int main(int argc, char** argv) {
  using namespace orb_slam3_wrapper;
  StereoCalibConfig config;
  const int parsed = parse(argc, argv, config);
  if (parsed == 0) return 0;
  if (parsed == 1) {
    printHelp(argv[0]);
    return 1;
  }
  try {
    std::cerr << "stereo_rotation_center_check\n"
              << "  bag:      " << config.bag_path.string() << '\n'
              << "  output:   " << config.output_dir.string() << '\n'
              << "  vocab:    " << config.vocabulary_file.string() << '\n'
              << "  settings: " << config.settings_file.string() << '\n';
    const auto run = runStereoCalibration(config);
    std::cerr << "[ 95%] writing report\n";
    writeStereoCalibrationReport(run);
    std::cerr << "[100%] done  classification="
              << resultClassName(run.aggregate.result_class)
              << "  median_xy=(" << run.aggregate.estimate.median_xy.x_m << ", "
              << run.aggregate.estimate.median_xy.y_m << ")\n";
    // Report is written before scientific exit codes (0/2/3).
    return resultExitCode(run.aggregate.result_class);
  } catch (const std::exception& error) {
    std::cerr << "\nstereo calibration operational failure: " << error.what()
              << '\n';
    return 1;
  } catch (...) {
    std::cerr << "\nstereo calibration operational failure: unknown error\n";
    return 1;
  }
}
