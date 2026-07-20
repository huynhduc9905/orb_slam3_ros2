#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/utility.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

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
  int configure_calls{0};
  bool configure_ok{true};
  int configure_failures{0};
  int track_calls{0};

  bool configureCalibration(const orb_slam3_wrapper::StereoCalibration&, std::string& error) override {
    ++configure_calls;
    if (!configure_ok || configure_failures-- > 0) { error = "fake calibration rejected"; return false; }
    return true;
  }

  ORB_SLAM3::FrameSnapshot trackStereo(
      const cv::Mat&, const cv::Mat&, double) override {
    ++track_calls;
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

sensor_msgs::msg::CameraInfo info(bool right = false) {
  sensor_msgs::msg::CameraInfo result;
  result.width = 848; result.height = 480;
  result.k[0] = result.k[4] = result.p[0] = result.p[5] = 426.9840393066406;
  result.k[2] = result.p[2] = 430.81121826171875;
  result.k[5] = result.p[6] = 238.95848083496094;
  result.k[8] = result.p[10] = 1.0;
  result.p[3] = right ? -21.429536819458008 : 0.0;
  result.header.frame_id = right ? "wrong_camera_info_frame" : "wrong_camera_info_frame";
  return result;
}

void setInfo(const std::shared_ptr<orb_slam3_wrapper::WrapperNode>& node) {
  node->setCameraInfoForTest(info(), info(true));
}

bool spinUntil(const std::shared_ptr<orb_slam3_wrapper::WrapperNode>& node,
               const std::function<bool()>& ready) {
  for (int i = 0; i < 20 && !ready(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return ready();
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
  setInfo(node);

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
  auto* backend_ptr = backend.get();
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  setInfo(node);

  sensor_msgs::msg::Image image;
  image.header.frame_id = "left_optical";
  image.header.stamp.sec = 2;
  image.height = image.width = 2;
  image.encoding = "mono8";
  image.step = 2;
  image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);

  EXPECT_EQ(node->graphPublishCountForTest(), 0u);
  backend_ptr->graph.revision = 4;
  backend_ptr->graph.active_map_id = 17;
  backend_ptr->changed = true;
  ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 1u; }));
  EXPECT_EQ(node->lastGraphSnapshotForTest().revision, 4u);
}

TEST_F(WrapperComponentTest, GraphLoopEdgesAreCanonicalizedAndDeduplicated) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame();
  backend->graph.revision = 1;
  backend->changed = true;
  auto* backend_ptr = backend.get();
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  setInfo(node);

  sensor_msgs::msg::Image image;
  image.header.frame_id = "left_optical";
  image.height = image.width = 2;
  image.encoding = "mono8";
  image.step = 2;
  image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);
  ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 1u; }));

  ORB_SLAM3::KeyframeSnapshot first;
  first.id = 20;
  first.map_id = 17;
  first.loop_edge_ids = {10};
  ORB_SLAM3::KeyframeSnapshot second;
  second.id = 10;
  second.map_id = 17;
  second.loop_edge_ids = {20};
  backend_ptr->graph.revision = 2;
  backend_ptr->graph.keyframes = {first, second};
  backend_ptr->changed = true;
  ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 2u; }));
  ASSERT_EQ(node->lastGraphSnapshotForTest().loop_edges.size(), 1u);
  EXPECT_EQ(node->lastGraphSnapshotForTest().loop_edges[0].from_id, 10u);
  EXPECT_EQ(node->lastGraphSnapshotForTest().loop_edges[0].to_id, 20u);
  EXPECT_EQ(node->lastTrackingEventForTest().type,
            orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED);
}

TEST_F(WrapperComponentTest, FirstChangedGraphWithLoopEdgeEmitsLoopClosed) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame();
  backend->graph.active_map_id = 17;
  auto* backend_ptr = backend.get();
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  setInfo(node);

  sensor_msgs::msg::Image image;
  image.header.frame_id = "left_optical";
  image.height = image.width = 2;
  image.encoding = "mono8";
  image.step = 2;
  image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  rclcpp::spin_some(node);
  EXPECT_EQ(node->graphPublishCountForTest(), 0u);
  EXPECT_EQ(node->eventPublishCountForTest(), 1u);

  ORB_SLAM3::KeyframeSnapshot first;
  first.id = 20;
  first.map_id = 17;
  first.loop_edge_ids = {10};
  ORB_SLAM3::KeyframeSnapshot second;
  second.id = 10;
  second.map_id = 17;
  second.loop_edge_ids = {20};
  backend_ptr->graph.revision = 1;
  backend_ptr->graph.active_map_id = 17;
  backend_ptr->graph.keyframes = {first, second};
  backend_ptr->changed = true;

  ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 1u; }));
  EXPECT_EQ(node->lastTrackingEventForTest().type,
            orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED);
  EXPECT_EQ(node->eventPublishCountForTest(), 2u);
}

TEST_F(WrapperComponentTest, ReconfigurationResetsGraphObservationSession) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame();
  auto* backend_ptr = backend.get();
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  setInfo(node);

  sensor_msgs::msg::Image image;
  image.header.frame_id = "left_optical";
  image.height = image.width = 2;
  image.encoding = "mono8";
  image.step = 2;
  image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);

  backend_ptr->graph.revision = 9;
  backend_ptr->graph.active_map_id = 17;
  backend_ptr->changed = true;
  ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 1u; }));

  auto changed_left = info();
  changed_left.width = 847;
  auto changed_right = info(true);
  changed_right.width = 847;
  node->setCameraInfoForTest(changed_left, changed_right);
  backend_ptr->changed = false;
  backend_ptr->graph.revision = 0;
  backend_ptr->graph.active_map_id = 23;
  backend_ptr->graph.keyframes.clear();
  node->processStereoForTest(image, image);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  rclcpp::spin_some(node);
  EXPECT_EQ(node->graphPublishCountForTest(), 1u);

  ORB_SLAM3::KeyframeSnapshot first;
  first.id = 20;
  first.map_id = 23;
  first.loop_edge_ids = {10};
  ORB_SLAM3::KeyframeSnapshot second;
  second.id = 10;
  second.map_id = 23;
  second.loop_edge_ids = {20};
  backend_ptr->graph.revision = 9;
  backend_ptr->graph.keyframes = {first, second};
  backend_ptr->changed = true;

  ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 2u; }));
  EXPECT_EQ(node->lastGraphSnapshotForTest().revision, 9u);
  EXPECT_EQ(node->lastTrackingEventForTest().type,
            orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED);
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
  setInfo(node);
  sensor_msgs::msg::Image image;
  image.header.frame_id = "left_optical"; image.height = image.width = 2;
  image.encoding = "mono8"; image.step = 2; image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);
  EXPECT_TRUE(node->lastTrackedFrameForTest().pose_valid);
  node->processStereoForTest(image, image);
  EXPECT_TRUE(node->lastTrackedFrameForTest().pose_valid);
  EXPECT_DOUBLE_EQ(node->lastTrackedFrameForTest().pose.position.x, 0.0);
}

TEST_F(WrapperComponentTest, MissingCameraInfoBlocksTracking) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame();
  auto* backend_ptr = backend.get();
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  sensor_msgs::msg::Image image; image.header.frame_id = "left_optical";
  image.height = image.width = 2; image.encoding = "mono8"; image.step = 2; image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);
  EXPECT_EQ(backend_ptr->configure_calls, 0);
  EXPECT_EQ(node->lastTrackedFrameForTest().map_id, 0u);
}

TEST_F(WrapperComponentTest, ValidCalibrationConfiguresBackendExactlyOnceAndCoheresRevision) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame(); backend->changed = true; backend->graph.revision = 9;
  auto* backend_ptr = backend.get();
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  setInfo(node);
  sensor_msgs::msg::Image image; image.header.frame_id = "left_optical";
  image.height = image.width = 2; image.encoding = "mono8"; image.step = 2; image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);
  ASSERT_TRUE(spinUntil(node, [&] { return node->graphPublishCountForTest() == 1u; }));
  node->processStereoForTest(image, image);
  EXPECT_EQ(backend_ptr->configure_calls, 1);
  EXPECT_EQ(node->lastTrackedFrameForTest().graph_revision, 9u);
}

TEST_F(WrapperComponentTest, RepeatedIdenticalCameraInfoDoesNotReconfigureBackend) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame();
  auto* backend_ptr = backend.get();
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  setInfo(node);
  sensor_msgs::msg::Image image; image.header.frame_id = "left_optical";
  image.height = image.width = 2; image.encoding = "mono8"; image.step = 2;
  image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);
  setInfo(node);
  node->processStereoForTest(image, image);
  EXPECT_EQ(backend_ptr->configure_calls, 1);
  EXPECT_EQ(backend_ptr->track_calls, 2);
}

TEST_F(WrapperComponentTest, BackendCalibrationRejectionBlocksTrackStereo) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame(); backend->configure_ok = false;
  auto* backend_ptr = backend.get();
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  setInfo(node);
  sensor_msgs::msg::Image image; image.header.frame_id = "left_optical";
  image.height = image.width = 2; image.encoding = "mono8"; image.step = 2; image.data = {0, 0, 0, 0};
  node->processStereoForTest(image, image);
  node->processStereoForTest(image, image);
  EXPECT_EQ(backend_ptr->configure_calls, 1);
  EXPECT_EQ(backend_ptr->track_calls, 0);
}

TEST_F(WrapperComponentTest, CorrectedImageFramesPermitConfigurationRetry) {
  auto backend = std::make_unique<FakeBackend>();
  backend->frame = okFrame(); backend->configure_failures = 1;
  auto* backend_ptr = backend.get();
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::move(backend));
  setInfo(node);
  sensor_msgs::msg::Image left; left.header.frame_id = "left_optical";
  sensor_msgs::msg::Image right = left; right.header.frame_id = "right_optical";
  left.height = right.height = left.width = right.width = 2;
  left.encoding = right.encoding = "mono8"; left.step = right.step = 2; left.data = right.data = {0,0,0,0};
  node->processStereoForTest(left, right);
  auto corrected_left = info();
  corrected_left.width = 847;
  auto corrected_right = info(true);
  corrected_right.width = 847;
  node->setCameraInfoForTest(corrected_left, corrected_right);
  node->processStereoForTest(left, right);
  EXPECT_EQ(backend_ptr->configure_calls, 2);
  EXPECT_EQ(backend_ptr->track_calls, 1);
}

TEST_F(WrapperComponentTest, OpenCvThreadCountDefaultsToFourAndIsConfigurable) {
  auto node = std::make_shared<orb_slam3_wrapper::WrapperNode>(std::make_unique<FakeBackend>());
  EXPECT_EQ(node->get_parameter("opencv_num_threads").as_int(), 4);
  EXPECT_EQ(cv::getNumThreads(), 4);
}

}  // namespace
