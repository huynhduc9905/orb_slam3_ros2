#include <gtest/gtest.h>
#include <System.h>

TEST(VendorLink, ExposesStereoSensor) {
  EXPECT_EQ(static_cast<int>(ORB_SLAM3::System::STEREO), 1);
}
