// mapper_node_test.cpp — synthetic integration test for MapperNode
// Pattern: construct node directly, drive inputs via real pub/sub, spin, assert outputs.
// Does NOT load ORB-SLAM3.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <orb_slam3_msgs/msg/graph_snapshot.hpp>
#include <orb_slam3_msgs/msg/keyframe_pose.hpp>
#include <orb_slam3_msgs/msg/map_revision.hpp>
#include <orb_slam3_msgs/msg/revisioned_path.hpp>
#include <orb_slam3_msgs/msg/tracked_frame.hpp>
#include <orb_slam3_msgs/msg/tracking_event.hpp>

#include "orb_lidar_mapper/mapper_node.hpp"

namespace orb_lidar_mapper {
namespace {

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Message factories
// ---------------------------------------------------------------------------

rclcpp::Time t(int sec) { return rclcpp::Time(sec, 0, RCL_ROS_TIME); }

std_msgs::msg::Header hdr(int sec, const std::string& frame = "orb_map") {
  std_msgs::msg::Header h;
  h.stamp = t(sec);
  h.frame_id = frame;
  return h;
}

nav_msgs::msg::Odometry odom(int sec, double x) {
  nav_msgs::msg::Odometry msg;
  msg.header = hdr(sec, "odom");
  msg.pose.pose.position.x = x;
  msg.pose.pose.orientation.w = 1.0;
  return msg;
}

sensor_msgs::msg::LaserScan make_scan(int sec, float range = 3.0f, int rays = 360) {
  sensor_msgs::msg::LaserScan msg;
  msg.header = hdr(sec, "base_scan");
  msg.angle_min = -static_cast<float>(M_PI);
  msg.angle_max =  static_cast<float>(M_PI);
  msg.angle_increment = static_cast<float>(2.0 * M_PI / rays);
  msg.time_increment = 0.0f;
  msg.range_min = 0.1f;
  msg.range_max = 15.0f;
  msg.ranges.assign(rays, range);
  return msg;
}

orb_slam3_msgs::msg::TrackedFrame make_tracked(int sec, uint8_t state, bool pose_valid,
                                               double x = 0.0,
                                               uint64_t map_id = 1,
                                               uint64_t kf_id = 1,
                                               uint64_t graph_rev = 1) {
  orb_slam3_msgs::msg::TrackedFrame msg;
  msg.header = hdr(sec, "orb_map");
  msg.tracking_state = state;
  msg.pose_valid = pose_valid;
  msg.map_id = map_id;
  msg.reference_keyframe_id = kf_id;
  msg.graph_revision = graph_rev;
  msg.pose.position.x = x;
  msg.pose.orientation.w = 1.0;
  msg.reference_to_frame.rotation.w = 1.0;
  return msg;
}

orb_slam3_msgs::msg::GraphSnapshot make_graph(int sec, uint64_t rev, uint64_t map_id,
                                              double kf_x = 0.0) {
  orb_slam3_msgs::msg::GraphSnapshot msg;
  msg.header = hdr(sec, "orb_map");
  msg.revision = rev;
  msg.active_map_id = map_id;
  msg.active_map_connected = true;
  orb_slam3_msgs::msg::KeyframePose kf;
  kf.id = 1; kf.map_id = map_id;
  kf.pose.position.x = kf_x;
  kf.pose.orientation.w = 1.0;
  msg.keyframes.push_back(kf);
  return msg;
}

orb_slam3_msgs::msg::TrackingEvent make_event(int sec, uint8_t type,
                                              uint64_t rev = 1, uint64_t map_id = 1) {
  orb_slam3_msgs::msg::TrackingEvent msg;
  msg.header = hdr(sec, "orb_map");
  msg.type = type;
  msg.graph_revision = rev;
  msg.map_id = map_id;
  return msg;
}

// ---------------------------------------------------------------------------
// Spin helpers
// ---------------------------------------------------------------------------

template<typename Pred>
bool spinUntil2(rclcpp::Node::SharedPtr a, rclcpp::Node::SharedPtr b, Pred pred,
                std::chrono::milliseconds timeout = 8000ms) {
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(a);
  exec.add_node(b);
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!pred() && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(20ms);
  }
  return pred();
}

void spinFlush(rclcpp::Node::SharedPtr a, rclcpp::Node::SharedPtr b,
               std::chrono::milliseconds dur = 150ms) {
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(a);
  exec.add_node(b);
  auto deadline = std::chrono::steady_clock::now() + dur;
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(20ms);
  }
}

// ---------------------------------------------------------------------------
// Test fixture — each test gets its own isolated MapperNode + helper
// ---------------------------------------------------------------------------

class MapperNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    // Unique node name suffix avoids duplicate-name warnings.
    const std::string suffix = std::to_string(instance_id_++);
    // Each test uses a unique graph revision base to avoid transient-local
    // topic pollution from previous tests' graph snapshots.
    graph_rev_base_ = static_cast<uint64_t>(instance_id_) * 1000u;
    helper_ = rclcpp::Node::make_shared("mapper_test_helper_" + suffix);

    // Static TF: base_link -> base_scan (identity — lidar at base origin).
    tf_bcast_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper_);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = rclcpp::Clock().now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "base_scan";
    tf.transform.rotation.w = 1.0;
    tf_bcast_->sendTransform(tf);

    auto sensor_qos  = rclcpp::SensorDataQoS();
    auto reliable_tl = rclcpp::QoS(1).reliable().transient_local();
    auto reliable    = rclcpp::QoS(100).reliable();
    auto reliable10  = rclcpp::QoS(10).reliable();

    odom_pub_    = helper_->create_publisher<nav_msgs::msg::Odometry>("/odom_wheel", sensor_qos);
    scan_pub_    = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/scan_origin", sensor_qos);
    tracked_pub_ = helper_->create_publisher<orb_slam3_msgs::msg::TrackedFrame>(
        "/orb_slam3/tracked_frame", sensor_qos);
    graph_pub_   = helper_->create_publisher<orb_slam3_msgs::msg::GraphSnapshot>(
        "/orb_slam3/graph_snapshot", reliable_tl);
    event_pub_   = helper_->create_publisher<orb_slam3_msgs::msg::TrackingEvent>(
        "/orb_slam3/events", reliable);

    map_sub_ = helper_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/orb_lidar/map", reliable_tl,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { last_map_ = msg; });
    map_rev_sub_ = helper_->create_subscription<orb_slam3_msgs::msg::MapRevision>(
        "/orb_lidar/map_revision", reliable_tl,
        [this](orb_slam3_msgs::msg::MapRevision::ConstSharedPtr msg) {
          last_map_revision_ = msg; ++map_rev_count_;
        });
    prov_sub_ = helper_->create_subscription<visualization_msgs::msg::Marker>(
        "/orb_lidar/provisional_scan", reliable10,
        [this](visualization_msgs::msg::Marker::ConstSharedPtr msg) {
          last_prov_ = msg;
        });
    wheel_path_sub_ = helper_->create_subscription<nav_msgs::msg::Path>(
        "/orb_lidar/wheel_path", reliable10,
        [this](nav_msgs::msg::Path::ConstSharedPtr msg) { last_wheel_path_ = msg; });
    corr_path_sub_ = helper_->create_subscription<nav_msgs::msg::Path>(
        "/orb_lidar/corrected_path", reliable_tl,
        [this](nav_msgs::msg::Path::ConstSharedPtr msg) { last_corr_path_ = msg; });

    // Create fresh MapperNode — default parameters use the global topic names above.
    mapper_ = std::make_shared<MapperNode>();

    // Let subscriptions discover each other before publishing.
    spinFlush(mapper_, helper_, 200ms);
  }

  void TearDown() override {
    mapper_.reset();
    helper_.reset();
    tf_bcast_.reset();
  }

  void publishWheelBurst(int from_sec, int to_sec, double x_per_sec = 0.1) {
    for (int s = from_sec; s <= to_sec; ++s) {
      odom_pub_->publish(odom(s, s * x_per_sec));
    }
  }

  static inline int instance_id_{0};
  uint64_t graph_rev_base_{1000};

  rclcpp::Node::SharedPtr helper_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_bcast_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<orb_slam3_msgs::msg::TrackedFrame>::SharedPtr tracked_pub_;
  rclcpp::Publisher<orb_slam3_msgs::msg::GraphSnapshot>::SharedPtr graph_pub_;
  rclcpp::Publisher<orb_slam3_msgs::msg::TrackingEvent>::SharedPtr event_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<orb_slam3_msgs::msg::MapRevision>::SharedPtr map_rev_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr prov_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr wheel_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr corr_path_sub_;

  std::shared_ptr<MapperNode> mapper_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr last_map_;
  orb_slam3_msgs::msg::MapRevision::ConstSharedPtr last_map_revision_;
  visualization_msgs::msg::Marker::ConstSharedPtr last_prov_;
  nav_msgs::msg::Path::ConstSharedPtr last_wheel_path_;
  nav_msgs::msg::Path::ConstSharedPtr last_corr_path_;
  std::size_t map_rev_count_{0};
};

// ---------------------------------------------------------------------------
// Test 1: OK tracking → map published with occupied and free cells
// ---------------------------------------------------------------------------
TEST_F(MapperNodeTest, OkTrackingProducesMapRevision1WithOccupiedAndFreeCells) {
  // Wheel odometry must arrive before the scan so interpolation works.
  publishWheelBurst(0, 5);
  spinFlush(mapper_, helper_, 300ms);

  // OK tracked frame — provides visual anchor.
  tracked_pub_->publish(make_tracked(2, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 0.0, 1, 1, 1));
  spinFlush(mapper_, helper_, 200ms);

  // Graph snapshot resolves keyframe → frame becomes committed.
  graph_pub_->publish(make_graph(2, 1, 1));
  spinFlush(mapper_, helper_, 300ms);

  // Scan at t=2s — should be committed and trigger incremental map publish.
  const std::size_t rev_count_before = map_rev_count_;
  scan_pub_->publish(make_scan(2, 2.0f));

  bool ok = spinUntil2(mapper_, helper_,
    [this, rev_count_before] {
      return map_rev_count_ > rev_count_before &&
             last_map_revision_ &&
             last_map_revision_->committed_scan_count >= 1u;
    });

  ASSERT_TRUE(ok) << "Timed out waiting for published map revision";
  ASSERT_NE(last_map_, nullptr);
  EXPECT_EQ(last_map_->header.frame_id, "orb_map");

  bool has_occ = false, has_free = false;
  for (int8_t c : last_map_->data) {
    if (c > 50)             has_occ  = true;
    if (c >= 0 && c <= 30)  has_free = true;
  }
  EXPECT_TRUE(has_occ)  << "Map has no occupied cells";
  EXPECT_TRUE(has_free) << "Map has no free cells";
}

// ---------------------------------------------------------------------------
// Corrected path poses must carry their own per-scan timestamp, not the
// single publish-time header stamp (needed for time-based cross-run matching).
// ---------------------------------------------------------------------------
TEST_F(MapperNodeTest, CorrectedPathPosesCarryPerScanStamps) {
  publishWheelBurst(0, 5);
  spinFlush(mapper_, helper_, 300ms);
  tracked_pub_->publish(make_tracked(2, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 0.0, 1, 1, 1));
  spinFlush(mapper_, helper_, 200ms);
  graph_pub_->publish(make_graph(2, 1, 1));
  spinFlush(mapper_, helper_, 300ms);

  scan_pub_->publish(make_scan(2, 2.0f));
  spinFlush(mapper_, helper_, 300ms);
  // A new graph revision after the scan is committed republishes the corrected
  // path, now containing the committed scan pose. (Same-revision snapshots are
  // deduplicated, so bump the revision.)
  graph_pub_->publish(make_graph(2, 2, 1));
  bool ok = spinUntil2(mapper_, helper_,
    [this] { return last_corr_path_ && !last_corr_path_->poses.empty(); });
  ASSERT_TRUE(ok) << "Timed out waiting for corrected path with poses";

  // The committed scan was stamped at t=2s; its pose must reflect that, not the
  // (much larger) publish wall-clock time carried by the path header.
  const auto& ps = last_corr_path_->poses.front();
  EXPECT_EQ(ps.header.frame_id, "orb_map");
  EXPECT_EQ(ps.header.stamp.sec, 2)
      << "corrected path pose should carry its per-scan stamp (2s), got "
      << ps.header.stamp.sec;
  // Distinct poses at different scan times must have distinct stamps: the path
  // header stamp (publish time) must differ from the per-pose stamp.
  EXPECT_NE(ps.header.stamp.sec, last_corr_path_->header.stamp.sec)
      << "per-pose stamp must differ from publish-time header stamp";
}

// ---------------------------------------------------------------------------
// Test 2: LOST tracking freezes map; provisional scan marker appears
// ---------------------------------------------------------------------------
TEST_F(MapperNodeTest, LostTrackingFreezesMapAndShowsProvisionalScanMarker) {
  const uint64_t rev1 = graph_rev_base_ + 1u;

  // ── baseline: get first PUBLISHED revision ──────────────────────────────
  publishWheelBurst(0, 5);
  spinFlush(mapper_, helper_, 300ms);
  tracked_pub_->publish(make_tracked(2, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 0.0, 1, 1, rev1));
  spinFlush(mapper_, helper_, 200ms);
  graph_pub_->publish(make_graph(2, rev1, 1));
  spinFlush(mapper_, helper_, 300ms);
  const std::size_t base_count_before = map_rev_count_;
  scan_pub_->publish(make_scan(2, 2.0f));

  bool got_base = spinUntil2(mapper_, helper_,
    [this, base_count_before] {
      return map_rev_count_ > base_count_before &&
             last_map_revision_ &&
             last_map_revision_->committed_scan_count >= 1u;
    });
  ASSERT_TRUE(got_base) << "Baseline map revision not received";

  const uint64_t frozen_rev = last_map_revision_->map_revision;

  // ── go LOST ─────────────────────────────────────────────────────────────
  event_pub_->publish(make_event(3, orb_slam3_msgs::msg::TrackingEvent::LOST, rev1));
  tracked_pub_->publish(make_tracked(3, orb_slam3_msgs::msg::TrackedFrame::LOST,
                                     false, 0.0, 1, 1, rev1));
  spinFlush(mapper_, helper_, 200ms);

  // Wheel + scan while lost → provisional marker, map must NOT advance.
  publishWheelBurst(3, 7, 0.1);
  spinFlush(mapper_, helper_, 100ms);
  scan_pub_->publish(make_scan(4, 2.5f));

  bool got_prov = spinUntil2(mapper_, helper_,
    [this] {
      return last_prov_ != nullptr &&
             last_prov_->action == visualization_msgs::msg::Marker::ADD;
    }, 5000ms);

  ASSERT_TRUE(got_prov) << "No provisional scan marker received while LOST";
  EXPECT_EQ(last_prov_->type, visualization_msgs::msg::Marker::POINTS);
  EXPECT_GT(last_prov_->color.r, 0.5f);  // yellow
  EXPECT_GT(last_prov_->color.g, 0.5f);
  EXPECT_LT(last_prov_->color.b, 0.5f);

  // Map revision must NOT have advanced.
  EXPECT_EQ(last_map_revision_->map_revision, frozen_rev)
      << "Map revision advanced while LOST";
}

// World-x of the rightmost occupied cell (cell index → world via origin/resolution).
std::optional<double> maxOccupiedWorldX(const nav_msgs::msg::OccupancyGrid& grid) {
  std::optional<double> max_x;
  const double res = grid.info.resolution;
  const double ox = grid.info.origin.position.x;
  const uint32_t w = grid.info.width;
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    if (grid.data[i] <= 50) continue;
    const uint32_t cx = static_cast<uint32_t>(i % w);
    const double wx = ox + (static_cast<double>(cx) + 0.5) * res;
    if (!max_x || wx > *max_x) max_x = wx;
  }
  return max_x;
}

// ---------------------------------------------------------------------------
// Test 3: RELOCALIZED + corrected graph → map revision advances AND occupied
// hit endpoint moves by approximately the keyframe correction (~1.0 m).
// ---------------------------------------------------------------------------
TEST_F(MapperNodeTest, RelocalizationTriggersRebuildAndMapRevision2) {
  const uint64_t rev1 = graph_rev_base_ + 1u;
  const uint64_t rev2 = graph_rev_base_ + 2u;
  constexpr double kCorrectionM = 1.0;

  // ── baseline rev 1 ───────────────────────────────────────────────────────
  publishWheelBurst(0, 5);
  spinFlush(mapper_, helper_, 300ms);
  tracked_pub_->publish(make_tracked(2, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 0.0, 1, 1, rev1));
  spinFlush(mapper_, helper_, 200ms);
  graph_pub_->publish(make_graph(2, rev1, 1, 0.0));
  spinFlush(mapper_, helper_, 300ms);
  scan_pub_->publish(make_scan(2, 2.0f));

  const std::size_t rev_count_before = map_rev_count_;
  bool got_rev1 = spinUntil2(mapper_, helper_,
    [this, rev_count_before] {
      return map_rev_count_ > rev_count_before &&
             last_map_revision_ &&
             last_map_revision_->committed_scan_count >= 1u;
    });
  ASSERT_TRUE(got_rev1) << "Timed out waiting for map revision 1";
  const uint64_t map_rev1 = last_map_revision_->map_revision;

  ASSERT_NE(last_map_, nullptr);
  const auto occ_x_rev1 = maxOccupiedWorldX(*last_map_);
  ASSERT_TRUE(occ_x_rev1.has_value()) << "No occupied cells after revision 1";

  // ── LOST (no extra scans — recovery proof is rebuild of the baseline scan) ─
  event_pub_->publish(make_event(3, orb_slam3_msgs::msg::TrackingEvent::LOST, rev1));
  tracked_pub_->publish(make_tracked(3, orb_slam3_msgs::msg::TrackedFrame::LOST,
                                     false, 0.0, 1, 1, rev1));
  spinFlush(mapper_, helper_, 200ms);

  // ── RELOCALIZED + corrected graph (keyframe +1.0 m) ──────────────────────
  event_pub_->publish(make_event(8, orb_slam3_msgs::msg::TrackingEvent::RELOCALIZED, rev2));
  spinFlush(mapper_, helper_, 100ms);

  // Wheel samples so the post-reloc tracked frame can interpolate (no identity poison).
  publishWheelBurst(8, 10, 0.1);
  spinFlush(mapper_, helper_, 100ms);

  tracked_pub_->publish(make_tracked(9, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, kCorrectionM, 1, 1, rev2));
  spinFlush(mapper_, helper_, 200ms);

  // Corrected keyframe shifted by ~1.0 m — rebuild must move the same occupied hit.
  // Do not publish a new scan: the recovery proof is the archived scan re-inserted
  // at the corrected pose, not incremental append of a later scan.
  graph_pub_->publish(make_graph(9, rev2, 1, kCorrectionM));

  bool got_rev2 = spinUntil2(mapper_, helper_,
    [this, map_rev1] {
      return last_map_revision_ &&
             last_map_revision_->map_revision > map_rev1;
    }, 12000ms);

  ASSERT_TRUE(got_rev2) << "Timed out waiting for map revision 2 after relocalization";
  EXPECT_GE(last_map_revision_->committed_scan_count, 1u);

  ASSERT_NE(last_map_, nullptr);
  const auto occ_x_rev2 = maxOccupiedWorldX(*last_map_);
  ASSERT_TRUE(occ_x_rev2.has_value()) << "Map has no occupied cells after rebuild";

  // Core recovery proof: rightmost occupied hit moved by ~correction amount.
  // Allow a few cells of tolerance (resolution 0.05 → ~0.25 m = 5 cells).
  const double dx = *occ_x_rev2 - *occ_x_rev1;
  EXPECT_NEAR(dx, kCorrectionM, 0.25)
      << "occupied max-x should move by ~" << kCorrectionM
      << " m (rev1 x=" << *occ_x_rev1 << " rev2 x=" << *occ_x_rev2 << ")";
}

// ---------------------------------------------------------------------------
// Test 4: disconnected atlas must not wipe the last published map
// ---------------------------------------------------------------------------
TEST_F(MapperNodeTest, DisconnectedAtlasKeepsLastPublishedMap) {
  const uint64_t rev1 = graph_rev_base_ + 1u;
  const uint64_t rev2 = graph_rev_base_ + 2u;

  publishWheelBurst(0, 5);
  spinFlush(mapper_, helper_, 300ms);
  tracked_pub_->publish(make_tracked(2, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 0.0, 1, 1, rev1));
  spinFlush(mapper_, helper_, 200ms);
  graph_pub_->publish(make_graph(2, rev1, 1, 0.0));
  spinFlush(mapper_, helper_, 300ms);

  const std::size_t rev_count_before = map_rev_count_;
  scan_pub_->publish(make_scan(2, 2.0f));
  bool got_base = spinUntil2(mapper_, helper_,
    [this, rev_count_before] {
      return map_rev_count_ > rev_count_before &&
             last_map_revision_ &&
             last_map_revision_->committed_scan_count >= 1u;
    });
  ASSERT_TRUE(got_base) << "Baseline map not received";
  ASSERT_NE(last_map_, nullptr);

  // Let the rebuilder settle (PUBLISHED → IDLE may still emit a status).
  spinFlush(mapper_, helper_, 400ms);

  const uint64_t frozen_map_rev = last_map_revision_->map_revision;
  const auto frozen_map = last_map_;
  const auto frozen_data = last_map_->data;
  const uint32_t frozen_w = last_map_->info.width;
  const uint32_t frozen_h = last_map_->info.height;
  const std::size_t rev_count_at_freeze = map_rev_count_;

  // Newer graph snapshot with disconnected active map — must freeze last good map.
  auto disconnected = make_graph(3, rev2, 1, 0.0);
  disconnected.active_map_connected = false;
  graph_pub_->publish(disconnected);
  spinFlush(mapper_, helper_, 800ms);

  // No new map_revision message after freeze (allow residual IDLE already counted).
  EXPECT_EQ(map_rev_count_, rev_count_at_freeze)
      << "Map revision message published after disconnected snapshot";
  ASSERT_NE(last_map_revision_, nullptr);
  EXPECT_EQ(last_map_revision_->map_revision, frozen_map_rev)
      << "Map revision advanced after disconnected snapshot";
  ASSERT_NE(last_map_, nullptr);
  EXPECT_EQ(last_map_->data, frozen_data)
      << "Map grid data changed after disconnected snapshot";
  EXPECT_EQ(last_map_->info.width, frozen_w);
  EXPECT_EQ(last_map_->info.height, frozen_h);
  (void)frozen_map;
}

// ---------------------------------------------------------------------------
// Test 5: map and map_revision publishers are reliable transient-local
// ---------------------------------------------------------------------------
TEST_F(MapperNodeTest, MapTopicsAreReliableTransientLocal) {
  const auto map_i = mapper_->get_publishers_info_by_topic("/orb_lidar/map");
  const auto rev_i = mapper_->get_publishers_info_by_topic("/orb_lidar/map_revision");

  ASSERT_EQ(map_i.size(), 1u);
  EXPECT_EQ(map_i[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(map_i[0].qos_profile().durability(),  rclcpp::DurabilityPolicy::TransientLocal);

  ASSERT_EQ(rev_i.size(), 1u);
  EXPECT_EQ(rev_i[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(rev_i[0].qos_profile().durability(),  rclcpp::DurabilityPolicy::TransientLocal);
}

// ---------------------------------------------------------------------------
// Test 6: No canonical /map or /tf publisher
// ---------------------------------------------------------------------------
TEST_F(MapperNodeTest, NoCanonicalMapTopicOrMapOdomTfPublished) {
  EXPECT_TRUE(mapper_->get_publishers_info_by_topic("/map").empty())
      << "Node must not publish /map";
  EXPECT_TRUE(mapper_->get_publishers_info_by_topic("/tf").empty())
      << "Node must not publish /tf";
}

// ---------------------------------------------------------------------------
// Test 7: Default parameters
// ---------------------------------------------------------------------------
TEST_F(MapperNodeTest, DefaultParametersAreDeclaredCorrectly) {
  EXPECT_EQ(mapper_->get_parameter("odom_topic").as_string(),    "/odom_wheel");
  EXPECT_EQ(mapper_->get_parameter("scan_topic").as_string(),    "/scan_origin");
  EXPECT_EQ(mapper_->get_parameter("tracked_frame_topic").as_string(),
            "/orb_slam3/tracked_frame");
  EXPECT_EQ(mapper_->get_parameter("graph_snapshot_topic").as_string(),
            "/orb_slam3/graph_snapshot");
  EXPECT_EQ(mapper_->get_parameter("tracking_event_topic").as_string(),
            "/orb_slam3/events");
  EXPECT_EQ(mapper_->get_parameter("map_topic").as_string(),     "/orb_lidar/map");
  EXPECT_EQ(mapper_->get_parameter("map_frame").as_string(),     "orb_map");
  EXPECT_EQ(mapper_->get_parameter("base_frame").as_string(),    "base_link");
  EXPECT_NEAR(mapper_->get_parameter("wheel_retention_s").as_double(),  300.0, 1e-9);
  EXPECT_NEAR(mapper_->get_parameter("wheel_max_gap_ms").as_double(),   100.0, 1e-9);
  EXPECT_NEAR(mapper_->get_parameter("resolution_m").as_double(),         0.05, 1e-9);
  EXPECT_NEAR(mapper_->get_parameter("usable_range_m").as_double(),       12.0, 1e-9);
  EXPECT_NEAR(mapper_->get_parameter("max_roll_pitch_deg").as_double(),   10.0, 1e-9);
  EXPECT_NEAR(mapper_->get_parameter("max_height_delta_m").as_double(),   0.15, 1e-9);
}

}  // namespace
}  // namespace orb_lidar_mapper
