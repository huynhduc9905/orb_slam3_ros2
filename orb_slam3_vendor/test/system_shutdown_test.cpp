#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "System.h"

TEST(SystemShutdown, DestructionAfterShutdownIsClean) {
  const char* vocabulary = std::getenv("ORB_TEST_VOCAB");
  const char* settings = std::getenv("ORB_TEST_SETTINGS");
  ASSERT_NE(vocabulary, nullptr);
  ASSERT_NE(settings, nullptr);

  {
    auto system = std::make_unique<ORB_SLAM3::System>(
        vocabulary, settings, ORB_SLAM3::System::STEREO, false);
    system->Shutdown();
  }
}

#ifdef ORB_SLAM3_SNAPSHOT_TESTING
class SystemConstructionFailure : public ::testing::TestWithParam<
    ORB_SLAM3::System::ConstructionFailpoint> {
protected:
  void TearDown() override {
    ORB_SLAM3::System::ClearConstructionFailpointForTesting();
  }
};

TEST_P(SystemConstructionFailure, ThrowsAndCleansUpPartialConstruction) {
  const char* vocabulary = std::getenv("ORB_TEST_VOCAB");
  const char* settings = std::getenv("ORB_TEST_SETTINGS");
  ASSERT_NE(vocabulary, nullptr);
  ASSERT_NE(settings, nullptr);

  ORB_SLAM3::System::SetConstructionFailpointForTesting(GetParam());
  EXPECT_THROW(
      {
        auto system = std::make_unique<ORB_SLAM3::System>(
            vocabulary, settings, ORB_SLAM3::System::STEREO, false);
      },
      std::runtime_error);
}

INSTANTIATE_TEST_SUITE_P(
    WorkerStages, SystemConstructionFailure,
    ::testing::Values(
        ORB_SLAM3::System::ConstructionFailpoint::AfterLocalMappingStart,
        ORB_SLAM3::System::ConstructionFailpoint::AfterLoopClosingStart,
        ORB_SLAM3::System::ConstructionFailpoint::BeforeCompletion));
#endif
