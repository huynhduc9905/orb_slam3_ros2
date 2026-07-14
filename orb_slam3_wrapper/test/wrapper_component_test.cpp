#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <orb_slam3_wrapper/backend.hpp>
#include <orb_slam3_wrapper/wrapper_node.hpp>

namespace {

class FakeBackend final : public orb_slam3_wrapper::SlamBackend {
public:
  ORB_SLAM3::FrameSnapshot frame;
  std::vector<ORB_SLAM3::FrameSnapshot> frames;
  std::size_t frame_index{0};
  ORB_SLAM3::GraphSnapshot graph;
  bool changed{false};

  ORB_SLAM3::FrameSnapshot trackStereo(
      const cv::Mat&, const cv::Mat&, double) override {
    if (frames.empty()) return frame;
    return frames[std::min(frame_index++, frames.size() - 1)];
  }
  bool mapChanged() override { return changed; }
  ORB_SLAM3::GraphSnapshot graphSnapshot() override { return graph; }
};

ORB_SLAM3::FrameSnapshot okFrame() {
  ORB_SLAM3::FrameSnapshot result;
  result.tracking_state = 2;
  result.pose_valid = true;
  result.map_id = 17;
  result.reference_keyframe_id = 23;
  result.tracked_keypoints = 99;
  result.T_world_camera = Sophus::SE3f();
  result.T_reference_camera_current_camera = Sophus::SE3f();
  return result;
}

class WrapperComponentTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(WrapperComponentTest, FakeOkFramePublishesIdentityAnchoredTrackedFrame) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame();
  auto* backend_ptr = backend.get();
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));

  sensor_msgs::msg::Image left;
  sensor_msgs::msg::Image right;
  left.header.frame_id = "left_optical";
  right.header.frame_id = "right_optical";
  left.header.stamp.sec = 1;
  right.header.stamp.sec = 1;
  left.height = right.height = 2;
  left.width = right.width = 2;
  left.encoding = right.encoding = "mono8";
  left.step = right.step = 2;
  left.data = right.data = {0, 0, 0, 0};
  node->processStereoForTest(left, right);

  ASSERT_EQ(node->lastTrackedFrameForTest().tracking_state,
            orb_slam3_msgs::msg::TrackedFrame::OK);
  const auto tracked = node->lastTrackedFrameForTest();
  EXPECT_TRUE(tracked.pose_valid);
  EXPECT_EQ(tracked.header.frame_id, "orb_map");
  EXPECT_EQ(tracked.map_id, 17u);
  EXPECT_EQ(tracked.reference_keyframe_id, 23u);
  EXPECT_DOUBLE_EQ(tracked.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(tracked.pose.orientation.w, 1.0);
  EXPECT_NE(backend_ptr, nullptr);
}

TEST_F(WrapperComponentTest, MapChangePublishesOneSnapshotAndConservativeEvent) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame();
  backend->changed = true;
  backend->graph.revision = 4;
  backend->graph.active_map_id = 17;
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));

  sensor_msgs::msg::Image image;
  image.header.frame_id = "left_optical";
  image.header.stamp.sec = 2;
  image.height = image.width = 2;
  image.encoding = "mono8";
  image.step = 2;
  image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);

  EXPECT_EQ(node->graphPublishCountForTest(), 1u);
  EXPECT_EQ(node->lastGraphSnapshotForTest().revision, 4u);
  ASSERT_EQ(node->eventPublishCountForTest(), 2u);
  EXPECT_EQ(node->lastTrackingEventForTest().type,
            orb_slam3_msgs::msg::TrackingEvent::MAP_CREATED);
}

TEST_F(WrapperComponentTest, GraphLoopEdgesAreCanonicalizedAndDeduplicated) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame(); backend->changed = true; backend->graph.revision = 8;
  ORB_SLAM3::KeyframeSnapshot first; first.id = 20; first.map_id = 17; first.loop_edge_ids = {10};
  ORB_SLAM3::KeyframeSnapshot second; second.id = 10; second.map_id = 17; second.loop_edge_ids = {20};
  backend->graph.keyframes = {first, second};
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  sensor_msgs::msg::Image image;
  image.header.frame_id = "left_optical"; image.height = image.width = 2;
  image.encoding = "mono8"; image.step = 2; image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);
  ASSERT_EQ(node->lastGraphSnapshotForTest().loop_edges.size(), 1u);
  EXPECT_EQ(node->lastGraphSnapshotForTest().loop_edges[0].from_id, 10u);
  EXPECT_EQ(node->lastGraphSnapshotForTest().loop_edges[0].to_id, 20u);
}

TEST_F(WrapperComponentTest, GraphPublisherIsReliableTransientLocalAndEventsDepthHundred) {
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::make_unique<FakeBackend>());
  const auto graph = node->get_publishers_info_by_topic("/orb_slam3/graph_snapshot");
  const auto events = node->get_publishers_info_by_topic("/orb_slam3/events");
  ASSERT_EQ(graph.size(), 1u); ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(graph[0].qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
  EXPECT_EQ(graph[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(graph[0].qos_profile().depth(), 1u);
  EXPECT_EQ(events[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(events[0].qos_profile().depth(), 100u);
}

TEST_F(WrapperComponentTest, RecentlyLostValidityIsPreservedButCannotAnchor) {
  auto backend = std::make_unique<FakeBackend>();
  auto lost = okFrame(); lost.tracking_state = 3; lost.pose_valid = true;
  lost.T_world_camera.translation().x() = 10.0F;
  auto ok = okFrame(); ok.T_world_camera.translation().x() = 11.0F;
  backend->frames = {lost, ok};
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  sensor_msgs::msg::Image image;
  image.header.frame_id = "left_optical"; image.height = image.width = 2;
  image.encoding = "mono8"; image.step = 2; image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);
  EXPECT_TRUE(node->lastTrackedFrameForTest().pose_valid);
  node->processStereoForTest(image, image);
  EXPECT_TRUE(node->lastTrackedFrameForTest().pose_valid);
  EXPECT_DOUBLE_EQ(node->lastTrackedFrameForTest().pose.position.x, 0.0);
}

}  // namespace
