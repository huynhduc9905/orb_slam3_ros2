#include <gtest/gtest.h>

#include <sstream>

#include <opencv2/core/mat.hpp>
#include <orb_slam3_wrapper/backend.hpp>

namespace {

constexpr double kBaseline = 0.05018814280629158;

cv::Mat transform(const std::string& matrix_data) {
  cv::Mat result(4, 4, CV_32F);
  std::stringstream values(matrix_data);
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      float value;
      char comma;
      values >> value;
      result.at<float>(row, col) = value;
      values >> comma;
    }
  }
  return result;
}

TEST(BackendGeometry, RejectsNonRigidRectifiedTransforms) {
  EXPECT_FALSE(orb_slam3_wrapper::validRectifiedStereoTransform(
      transform("1.1, 0.0, 0.0, 0.05018814280629158, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0"), kBaseline));
  EXPECT_FALSE(orb_slam3_wrapper::validRectifiedStereoTransform(
      transform("1.0, 0.1, 0.0, 0.05018814280629158, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0"), kBaseline));
  EXPECT_FALSE(orb_slam3_wrapper::validRectifiedStereoTransform(
      transform("-1.0, 0.0, 0.0, 0.05018814280629158, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0"), kBaseline));
}

TEST(BackendGeometry, RejectsTranslationOutsidePositiveBaselineAxis) {
  EXPECT_FALSE(orb_slam3_wrapper::validRectifiedStereoTransform(
      transform("1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.05018814280629158, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0"), kBaseline));
  EXPECT_FALSE(orb_slam3_wrapper::validRectifiedStereoTransform(
      transform("1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.05018814280629158, 0.0, 0.0, 0.0, 1.0"), kBaseline));
  EXPECT_FALSE(orb_slam3_wrapper::validRectifiedStereoTransform(
      transform("1.0, 0.0, 0.0, -0.05018814280629158, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0"), kBaseline));
}

TEST(BackendGeometry, AcceptsTinyFloatingPointNoise) {
  EXPECT_TRUE(orb_slam3_wrapper::validRectifiedStereoTransform(
      transform("1.0000002, 0.0000002, 0.0, 0.05018814280629158, 0.0, 0.9999998, 0.0000002, 0.0000002, 0.0, 0.0, 1.0000002, 0.0, 0.0, 0.0, 0.0, 1.0"), kBaseline));
}

}  // namespace
