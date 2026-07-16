#include "orb_lidar_mapper/calibration_report.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace orb_lidar_mapper {
namespace {

void printHelp(const char* executable) {
  std::cout << "Usage: " << executable << " --bag PATH --output DIR [OPTIONS]\n"
            << "Options:\n"
            << "  --bag PATH\n"
            << "  --output DIR\n"
            << "  --overwrite\n"
            << "  --range-cap-m M\n"
            << "  --min-omega RAD_S\n"
            << "  --max-omega RAD_S\n"
            << "  --max-linear-speed M_S\n"
            << "  --help\n";
}

bool readDouble(int argc, char** argv, int& index, double& destination) {
  if (index + 1 >= argc) return false;
  try {
    std::size_t consumed = 0;
    const std::string value = argv[++index];
    destination = std::stod(value, &consumed);
    return consumed == value.size() && std::isfinite(destination);
  } catch (...) {
    return false;
  }
}

int parse(int argc, char** argv, CalibrationConfig& config) {
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
    } else if (option == "--bag" || option == "--output") {
      if (index + 1 >= argc) return 1;
      const std::string value = argv[++index];
      if (value.empty() || value.rfind("--", 0) == 0) return 1;
      if (option == "--bag") {
        config.bag_path = value;
        have_bag = true;
      } else {
        config.output_dir = value;
        have_output = true;
      }
    } else if (option == "--range-cap-m") {
      if (!readDouble(argc, argv, index, config.range_cap_m) || config.range_cap_m < 0.0) return 1;
    } else if (option == "--min-omega") {
      if (!readDouble(argc, argv, index, config.min_abs_omega) || config.min_abs_omega < 0.0) return 1;
    } else if (option == "--max-omega") {
      if (!readDouble(argc, argv, index, config.max_abs_omega) || config.max_abs_omega < 0.0) return 1;
    } else if (option == "--max-linear-speed") {
      if (!readDouble(argc, argv, index, config.max_abs_linear_speed) || config.max_abs_linear_speed < 0.0) return 1;
    } else {
      return 1;
    }
  }
  if (!have_bag || !have_output || config.max_abs_omega < config.min_abs_omega) return 1;
  return -1;
}

}  // namespace
}  // namespace orb_lidar_mapper

int main(int argc, char** argv) {
  using namespace orb_lidar_mapper;
  CalibrationConfig config;
  const int parsed = parse(argc, argv, config);
  if (parsed == 0) return 0;
  if (parsed == 1) {
    printHelp(argv[0]);
    return 1;
  }
  try {
    const auto run = runCalibration(config);
    writeCalibrationReport(run);
    return resultExitCode(run.aggregate.classification);
  } catch (const std::exception& error) {
    std::cerr << "calibration operational failure: " << error.what() << '\n';
    return 1;
  }
}
