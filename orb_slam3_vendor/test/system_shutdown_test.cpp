#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

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
