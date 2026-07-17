#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <Eigen/Geometry>

#include "orb_slam3_wrapper/stereo_bag_reader.hpp"

namespace orb_slam3_wrapper {
namespace {

geometry_msgs::msg::TransformStamped makeTf(
    const std::string& parent, const std::string& child,
    double x, double y, double z, double yaw_rad = 0.0) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  t.transform.rotation.z = std::sin(yaw_rad / 2.0);
  t.transform.rotation.w = std::cos(yaw_rad / 2.0);
  return t;
}

// ── TF composition helpers (no bag) ─────────────────────────────────────────

TEST(StereoBagReaderTf, IsometryFromGeometryTransform) {
  geometry_msgs::msg::Transform tf;
  tf.translation.x = 0.3;
  tf.translation.y = 0.05;
  tf.translation.z = 0.1;
  tf.rotation.z = std::sin(0.25);
  tf.rotation.w = std::cos(0.25);
  const auto iso = isometryFromGeometryTransform(tf);
  EXPECT_NEAR(iso.translation().x(), 0.3, 1e-12);
  EXPECT_NEAR(iso.translation().y(), 0.05, 1e-12);
  EXPECT_NEAR(iso.translation().z(), 0.1, 1e-12);
  const Eigen::Quaterniond q(iso.linear());
  EXPECT_NEAR(q.z(), std::sin(0.25), 1e-12);
  EXPECT_NEAR(q.w(), std::cos(0.25), 1e-12);
}

TEST(StereoBagReaderTf, DirectBaseToCameraLink) {
  std::vector<geometry_msgs::msg::TransformStamped> tfs = {
      makeTf("base_link", "camera_link", 0.346, 0.01, 0.1),
      makeTf("camera_link", "camera_infra1_optical_frame", 0.0, 0.0, 0.0,
             -kPi / 2),
  };
  const auto mount = buildStaticCameraMount(tfs);
  EXPECT_NEAR(mount.T_base_camera_link.translation().x(), 0.346, 1e-12);
  EXPECT_NEAR(mount.T_base_camera_link.translation().y(), 0.01, 1e-12);
  EXPECT_NEAR(mount.T_base_camera_link.translation().z(), 0.1, 1e-12);
  // optical chain is non-identity (yaw -pi/2)
  EXPECT_GT(mount.T_camera_link_left_optical.linear().norm(), 0.5);
}

TEST(StereoBagReaderTf, MultiHopOpticalChain) {
  // camera_link → camera_infra1_frame → camera_infra1_optical_frame
  std::vector<geometry_msgs::msg::TransformStamped> tfs = {
      makeTf("base_link", "camera_link", 0.32, 0.05, 0.12),
      makeTf("camera_link", "camera_infra1_frame", 0.01, 0.0, 0.02),
      makeTf("camera_infra1_frame", "camera_infra1_optical_frame", 0.0, 0.0,
             0.0, -kPi / 2),
  };
  const auto mount = buildStaticCameraMount(tfs);
  EXPECT_NEAR(mount.T_base_camera_link.translation().x(), 0.32, 1e-12);
  // composed optical: t should include the intermediate 0.01, 0, 0.02
  // under identity intermediate rotation, then -pi/2 yaw
  const auto T = mount.T_camera_link_left_optical;
  // translation of multi-hop: first hop only for pure translation chain before rot
  // T = T_link_infra1 * T_infra1_optical
  // T_infra1_optical has zero translation, so T.translation = R(-pi/2)*0 + t_link_infra1
  // with identity R on first hop: translation = (0.01, 0, 0.02)
  EXPECT_NEAR(T.translation().x(), 0.01, 1e-9);
  EXPECT_NEAR(T.translation().y(), 0.0, 1e-9);
  EXPECT_NEAR(T.translation().z(), 0.02, 1e-9);
}

TEST(StereoBagReaderTf, MissingBaseToCameraLinkFailsClosed) {
  std::vector<geometry_msgs::msg::TransformStamped> tfs = {
      makeTf("base_link", "base_scan", 0.26, 0.0, 0.1, kPi),
      makeTf("camera_link", "camera_infra1_optical_frame", 0.0, 0.0, 0.0),
  };
  EXPECT_THROW(buildStaticCameraMount(tfs), std::runtime_error);
}

TEST(StereoBagReaderTf, MissingOpticalChainFailsClosed) {
  std::vector<geometry_msgs::msg::TransformStamped> tfs = {
      makeTf("base_link", "camera_link", 0.32, 0.05, 0.12),
  };
  EXPECT_THROW(buildStaticCameraMount(tfs), std::runtime_error);
}

// ── Stereo stamp sync ───────────────────────────────────────────────────────

TEST(StereoBagReaderSync, PairsWithinFiveMs) {
  // left @ 0, 10ms, 20ms; right @ 2ms, 11ms, 25ms (25-20 = 5ms boundary)
  const std::vector<int64_t> left = {0, 10'000'000LL, 20'000'000LL};
  const std::vector<int64_t> right = {2'000'000LL, 11'000'000LL, 25'000'000LL};
  const auto pairs = syncStereoStamps(left, right, 5'000'000LL);
  ASSERT_EQ(pairs.size(), 3U);
  EXPECT_EQ(std::get<0>(pairs[0]), 0U);
  EXPECT_EQ(std::get<1>(pairs[0]), 0U);
  EXPECT_EQ(std::get<0>(pairs[1]), 1U);
  EXPECT_EQ(std::get<1>(pairs[1]), 1U);
  EXPECT_EQ(std::get<0>(pairs[2]), 2U);
  EXPECT_EQ(std::get<1>(pairs[2]), 2U);
}

TEST(StereoBagReaderSync, IncludesExactFiveMsBoundary) {
  const std::vector<int64_t> left = {0};
  const std::vector<int64_t> right = {5'000'000LL};
  const auto pairs = syncStereoStamps(left, right, 5'000'000LL);
  ASSERT_EQ(pairs.size(), 1U);
  EXPECT_EQ(std::get<1>(pairs[0]), 0U);
}

TEST(StereoBagReaderSync, RejectsBeyondFiveMs) {
  const std::vector<int64_t> left = {0};
  const std::vector<int64_t> right = {5'000'001LL};
  const auto pairs = syncStereoStamps(left, right, 5'000'000LL);
  EXPECT_TRUE(pairs.empty());
}

TEST(StereoBagReaderSync, EachRightUsedAtMostOnce) {
  // two lefts both closer to same right → only nearest left claims it
  const std::vector<int64_t> left = {0, 1'000'000LL};
  const std::vector<int64_t> right = {500'000LL};
  const auto pairs = syncStereoStamps(left, right, 5'000'000LL);
  ASSERT_EQ(pairs.size(), 1U);
  EXPECT_EQ(std::get<0>(pairs[0]), 0U);  // first left is equally? 500k vs 500k
  // left0: |500k-0|=500k, left1: |500k-1M|=500k — greedy left-order takes left0
  EXPECT_EQ(std::get<1>(pairs[0]), 0U);
}

// ── Synthetic bag I/O ───────────────────────────────────────────────────────

class TempBag {
 public:
  explicit TempBag(std::filesystem::path path) : path_(std::move(path)) {}
  ~TempBag() { std::filesystem::remove_all(path_); }
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

void setStamp(builtin_interfaces::msg::Time& stamp, int64_t stamp_ns) {
  stamp.sec = static_cast<int32_t>(stamp_ns / 1'000'000'000LL);
  stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1'000'000'000LL);
}

sensor_msgs::msg::Image makeMonoImage(int64_t stamp_ns, const std::string& frame,
                                      uint8_t fill, int w = 4, int h = 2) {
  sensor_msgs::msg::Image img;
  setStamp(img.header.stamp, stamp_ns);
  img.header.frame_id = frame;
  img.height = static_cast<uint32_t>(h);
  img.width = static_cast<uint32_t>(w);
  img.encoding = "mono8";
  img.is_bigendian = 0;
  img.step = static_cast<uint32_t>(w);
  img.data.assign(static_cast<std::size_t>(w * h), fill);
  return img;
}

sensor_msgs::msg::CameraInfo makeInfo(int64_t stamp_ns, const std::string& frame,
                                      uint32_t w = 4, uint32_t h = 2) {
  sensor_msgs::msg::CameraInfo info;
  setStamp(info.header.stamp, stamp_ns);
  info.header.frame_id = frame;
  info.width = w;
  info.height = h;
  info.k = {100.0, 0.0, 2.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0};
  info.p = {100.0, 0.0, 2.0, 0.0, 0.0, 100.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  info.distortion_model = "plumb_bob";
  return info;
}

struct BagOptions {
  bool include_mount = true;
  bool include_images = true;
  bool include_odom = false;
  bool multi_hop_optical = false;
  double mount_x = 0.346;
  double mount_y = 0.01;
  double mount_z = 0.1;
  // left stamps / right stamps
  std::vector<int64_t> left_stamps = {1'000'000'000LL, 1'010'000'000LL};
  std::vector<int64_t> right_stamps = {1'001'000'000LL, 1'011'000'000LL};
};

TempBag writeStereoBag(const BagOptions& options = {}) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("stereo-calib-" + std::to_string(::getpid()) + "-" +
                     std::to_string(std::rand()));
  std::filesystem::remove_all(path);
  rosbag2_cpp::Writer writer;
  writer.open({path.string(), "mcap"}, {"cdr", "cdr"});

  if (options.include_images) {
    for (std::size_t i = 0; i < options.left_stamps.size(); ++i) {
      auto left = makeMonoImage(options.left_stamps[i],
                                "camera_infra1_optical_frame",
                                static_cast<uint8_t>(10 + i));
      writer.write(left, "/camera/camera/infra1/image_rect_raw",
                   rclcpp::Time(options.left_stamps[i]));
    }
    for (std::size_t i = 0; i < options.right_stamps.size(); ++i) {
      auto right = makeMonoImage(options.right_stamps[i],
                                 "camera_infra2_optical_frame",
                                 static_cast<uint8_t>(50 + i));
      writer.write(right, "/camera/camera/infra2/image_rect_raw",
                   rclcpp::Time(options.right_stamps[i]));
    }
    auto li = makeInfo(options.left_stamps.front(),
                       "camera_infra1_optical_frame");
    auto ri = makeInfo(options.right_stamps.front(),
                       "camera_infra2_optical_frame");
    writer.write(li, "/camera/camera/infra1/camera_info",
                 rclcpp::Time(options.left_stamps.front()));
    writer.write(ri, "/camera/camera/infra2/camera_info",
                 rclcpp::Time(options.right_stamps.front()));
  }

  {
    tf2_msgs::msg::TFMessage tf;
    if (options.include_mount) {
      tf.transforms.push_back(makeTf("base_link", "camera_link", options.mount_x,
                                     options.mount_y, options.mount_z));
      if (options.multi_hop_optical) {
        tf.transforms.push_back(
            makeTf("camera_link", "camera_infra1_frame", 0.01, 0.0, 0.02));
        tf.transforms.push_back(makeTf("camera_infra1_frame",
                                       "camera_infra1_optical_frame", 0.0, 0.0,
                                       0.0, -kPi / 2));
      } else {
        tf.transforms.push_back(
            makeTf("camera_link", "camera_infra1_optical_frame", 0.0, 0.0, 0.0,
                   -kPi / 2));
      }
      tf.transforms.push_back(
          makeTf("camera_link", "camera_infra2_optical_frame", 0.05, 0.0, 0.0,
                 -kPi / 2));
    }
    writer.write(tf, "/tf_static", rclcpp::Time(0));
  }

  if (options.include_odom) {
    nav_msgs::msg::Odometry odom;
    setStamp(odom.header.stamp, 1'000'000'000LL);
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = 1.0;
    odom.pose.pose.position.y = 2.0;
    odom.pose.pose.orientation.w = 1.0;
    writer.write(odom, "/odom", rclcpp::Time(1'000'000'000LL));
  }

  return TempBag(path);
}

class RclcppEnvironment : public ::testing::Environment {
 public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

// Register once so bag I/O and optional smoke share one init/shutdown pair.
::testing::Environment* const g_rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

TEST(StereoBagIo, ReadsSyncedFramesMountAndCameraInfo) {
  const auto bag = writeStereoBag();
  const auto data = StereoBagReader::read(bag.path());
  ASSERT_EQ(data.frames.size(), 2U);
  EXPECT_EQ(data.frames[0].stamp_ns, 1'000'000'000LL);
  EXPECT_EQ(data.frames[0].left_bgr_or_gray.type(), CV_8UC1);
  EXPECT_EQ(data.frames[0].right_bgr_or_gray.type(), CV_8UC1);
  EXPECT_EQ(data.frames[0].left_bgr_or_gray.at<uint8_t>(0, 0), 10);
  EXPECT_EQ(data.frames[0].right_bgr_or_gray.at<uint8_t>(0, 0), 50);
  EXPECT_NEAR(data.recorded_camera_link_xy.x_m, 0.346, 1e-12);
  EXPECT_NEAR(data.recorded_camera_link_xy.y_m, 0.01, 1e-12);
  EXPECT_NEAR(data.recorded_mount.T_base_camera_link.translation().x(), 0.346,
              1e-12);
  EXPECT_EQ(data.left_info.width, 4U);
  EXPECT_EQ(data.right_info.width, 4U);
  EXPECT_EQ(data.left_optical_frame, "camera_infra1_optical_frame");
  EXPECT_EQ(data.right_optical_frame, "camera_infra2_optical_frame");
  EXPECT_TRUE(data.odom_se2.empty());
}

TEST(StereoBagIo, MissingMountFailsClosed) {
  BagOptions options;
  options.include_mount = false;
  EXPECT_THROW(StereoBagReader::read(writeStereoBag(options).path()),
               std::runtime_error);
}

TEST(StereoBagIo, MultiHopOpticalFromBag) {
  BagOptions options;
  options.multi_hop_optical = true;
  const auto data = StereoBagReader::read(writeStereoBag(options).path());
  EXPECT_NEAR(data.recorded_mount.T_camera_link_left_optical.translation().x(),
              0.01, 1e-9);
  EXPECT_NEAR(data.recorded_mount.T_camera_link_left_optical.translation().z(),
              0.02, 1e-9);
}

TEST(StereoBagIo, OptionalOdomSe2) {
  BagOptions options;
  options.include_odom = true;
  const auto data = StereoBagReader::read(writeStereoBag(options).path());
  ASSERT_EQ(data.odom_se2.size(), 1U);
  EXPECT_EQ(data.odom_se2[0].first, 1'000'000'000LL);
  EXPECT_NEAR(data.odom_se2[0].second.x, 1.0, 1e-12);
  EXPECT_NEAR(data.odom_se2[0].second.y, 2.0, 1e-12);
  EXPECT_NEAR(data.odom_se2[0].second.yaw, 0.0, 1e-12);
}

TEST(StereoBagIo, DropsUnsyncedRightOutsideSkew) {
  BagOptions options;
  options.left_stamps = {1'000'000'000LL};
  options.right_stamps = {1'010'000'000LL};  // 10 ms — outside 5 ms
  const auto data = StereoBagReader::read(writeStereoBag(options).path());
  EXPECT_TRUE(data.frames.empty());
}

TEST(StereoBagReaderSmoke, OptionalRealBag) {
  const char* bag_env = std::getenv("STEREO_CALIB_BAG");
  if (bag_env == nullptr || bag_env[0] == '\0') {
    GTEST_SKIP() << "STEREO_CALIB_BAG not set";
  }
  const auto data = StereoBagReader::read(bag_env);
  EXPECT_FALSE(data.frames.empty());
  EXPECT_NEAR(data.recorded_camera_link_xy.x_m,
              data.recorded_mount.T_base_camera_link.translation().x(), 1e-12);
  EXPECT_FALSE(data.left_optical_frame.empty());
}

}  // namespace
}  // namespace orb_slam3_wrapper
