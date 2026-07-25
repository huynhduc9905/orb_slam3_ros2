// mapper_node_test.cpp — synthetic integration test for MapperNode
// Pattern: construct node directly, drive inputs via real pub/sub, spin, assert outputs.
// Does NOT load ORB-SLAM3.

#include <gtest/gtest.h>

#include <chrono>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
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

namespace mapper_node_test_hooks {
void armPublishBarrier();
bool waitForPublishBarrier(std::chrono::milliseconds timeout);
bool waitForDestructorEntry(std::chrono::milliseconds timeout);
void releasePublishBarrier();
bool publishCompleted();
void resetPublishBarrier();
}  // namespace mapper_node_test_hooks

namespace {

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Message factories
// ---------------------------------------------------------------------------

rclcpp::Time t(int sec) { return rclcpp::Time(sec, 0, RCL_ROS_TIME); }

rclcpp::Time tn(std::int64_t nanoseconds) {
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

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

nav_msgs::msg::Odometry odom_ns(std::int64_t stamp_ns, double x, double yaw = 0.0) {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = tn(stamp_ns);
  msg.header.frame_id = "odom";
  msg.pose.pose.position.x = x;
  msg.pose.pose.orientation.w = std::cos(yaw * 0.5);
  msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
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

sensor_msgs::msg::LaserScan sweep(
    std::int64_t start_ns, float time_increment,
    std::initializer_list<float> ranges) {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = tn(start_ns);
  msg.header.frame_id = "base_scan";
  msg.angle_min = -0.1F;
  msg.angle_increment = 0.1F;
  msg.time_increment = time_increment;
  msg.range_min = 0.1F;
  msg.range_max = 20.0F;
  msg.ranges.assign(ranges);
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

orb_slam3_msgs::msg::TrackedFrame tracked_ns(
    std::int64_t stamp_ns, uint8_t state, bool pose_valid, double x,
    uint64_t map_id, uint64_t kf_id, uint64_t graph_rev) {
  auto msg = make_tracked(0, state, pose_valid, x, map_id, kf_id, graph_rev);
  msg.header.stamp = tn(stamp_ns);
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
    auto map_history_qos = rclcpp::QoS(10).reliable().transient_local();
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
        "/orb_lidar/map", map_history_qos,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          last_map_ = msg;
          map_stamps_.push_back(msg->header.stamp);
          maps_.push_back(*msg);
        });
    map_rev_sub_ = helper_->create_subscription<orb_slam3_msgs::msg::MapRevision>(
        "/orb_lidar/map_revision", map_history_qos,
        [this](orb_slam3_msgs::msg::MapRevision::ConstSharedPtr msg) {
          last_map_revision_ = msg;
          map_revision_stamps_.push_back(msg->header.stamp);
          map_revisions_.push_back(*msg);
          if (msg->state == orb_slam3_msgs::msg::MapRevision::PUBLISHED) {
            last_published_map_revision_ = msg;
          }
          ++map_rev_count_;
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
    diagnostics_sub_ = helper_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", reliable10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
          diagnostics_.push_back(msg);
        });

    // Create fresh MapperNode — default parameters use the global topic names above.
    // Tests that rely on incremental map publication need rebuild_only_map=false;
    // production default is true (rebuild-only for map accuracy).
    rclcpp::NodeOptions default_opts;
    default_opts.parameter_overrides({
        rclcpp::Parameter("rebuild_only_map", false),
        rclcpp::Parameter("map_rebuild_min_interval_s", 0.0),
    });
    mapper_ = std::make_shared<MapperNode>(default_opts);

    // Let subscriptions discover each other before publishing.
    spinFlush(mapper_, helper_, 200ms);
  }

  void TearDown() override {
    mapper_node_test_hooks::releasePublishBarrier();
    mapper_node_test_hooks::resetPublishBarrier();
    mapper_.reset();
    helper_.reset();
    tf_bcast_.reset();
  }

  void publishWheelBurst(int from_sec, int to_sec, double x_per_sec = 0.1) {
    for (int s = from_sec; s <= to_sec; ++s) {
      odom_pub_->publish(odom(s, s * x_per_sec));
    }
  }

  void publishStraightSweepWithTwoVisualAnchors() {
    constexpr std::int64_t kStartNs = 2'000'000'000LL;
    const uint64_t revision = graph_rev_base_ + 1u;
    endpoint_counts_before_sweep_ = endpointCounts();
    for (int i = -1; i <= 3; ++i) {
      odom_pub_->publish(odom_ns(kStartNs + i * 50'000'000LL, 0.01 * i));
    }
    spinFlush(mapper_, helper_, 200ms);
    tracked_pub_->publish(tracked_ns(1'950'000'000LL,
        orb_slam3_msgs::msg::TrackedFrame::OK, true, -0.01, 1, 1, revision));
    spinFlush(mapper_, helper_, 100ms);
    tracked_pub_->publish(tracked_ns(2'150'000'000LL,
        orb_slam3_msgs::msg::TrackedFrame::OK, true, 0.03, 1, 1, revision));
    spinFlush(mapper_, helper_, 100ms);
    graph_pub_->publish(make_graph(3, revision, 1));
    spinFlush(mapper_, helper_, 300ms);
    scan_pub_->publish(sweep(kStartNs, 0.05F, {2.0F, 2.0F, 2.0F}));
    // Drive a corrected rebuild after the queued sweep has been accepted. This
    // exercises the same publication pair without racing its first incremental
    // worker wake-up against test teardown.
    spinFlush(mapper_, helper_, 200ms);
    graph_pub_->publish(make_graph(3, revision + 1u, 1));
    spinFlush(mapper_, helper_, 200ms);
  }

  void publishSweepWheelCoverage(double middle_yaw = 0.0, double final_yaw = 0.0,
                                 bool include_final = true, double x_step = 0.01) {
    constexpr std::int64_t kStartNs = 2'000'000'000LL;
    odom_pub_->publish(odom_ns(1'950'000'000LL, -x_step, 0.0));
    odom_pub_->publish(odom_ns(kStartNs, 0.0, 0.0));
    odom_pub_->publish(odom_ns(2'050'000'000LL, x_step, middle_yaw));
    if (include_final) {
      odom_pub_->publish(odom_ns(2'100'000'000LL, 2.0 * x_step, final_yaw));
      odom_pub_->publish(odom_ns(2'150'000'000LL, 3.0 * x_step, final_yaw));
    }
  }

  void publishVisualBracket(uint64_t revision, bool include_end = true,
                            double x_step = 0.01) {
    tracked_pub_->publish(tracked_ns(1'950'000'000LL,
        orb_slam3_msgs::msg::TrackedFrame::OK, true, -x_step, 1, 1, revision));
    if (include_end) {
      tracked_pub_->publish(tracked_ns(2'150'000'000LL,
          orb_slam3_msgs::msg::TrackedFrame::OK, true, 3.0 * x_step, 1, 1, revision));
    }
    graph_pub_->publish(make_graph(3, revision, 1));
  }

  void publishPreparedSweep(uint64_t revision, double middle_yaw = 0.0,
                            double final_yaw = 0.0, double x_step = 0.01) {
    publishSweepWheelCoverage(middle_yaw, final_yaw, true, x_step);
    publishVisualBracket(revision, true, x_step);
    spinFlush(mapper_, helper_, 200ms);
    scan_pub_->publish(sweep(2'000'000'000LL, 0.05F, {2.0F, 2.0F, 2.0F}));
  }

  bool waitForCommittedScanCount(uint32_t count,
                                 std::chrono::milliseconds timeout = 4000ms) {
    return spinUntil2(mapper_, helper_, [this, count] {
      return last_map_revision_ && last_map_revision_->committed_scan_count >= count;
    }, timeout);
  }

  std::optional<uint64_t> latestDiagnosticValue(const std::string& key) const {
    for (auto array = diagnostics_.rbegin(); array != diagnostics_.rend(); ++array) {
      for (const auto& status : (*array)->status) {
        for (const auto& value : status.values) {
          if (value.key == key) return std::stoull(value.value);
        }
      }
    }
    return std::nullopt;
  }

  bool waitForDiagnosticAtLeast(const std::string& key, uint64_t value,
                                std::chrono::milliseconds timeout = 4000ms) {
    return spinUntil2(mapper_, helper_, [this, &key, value] {
      const auto current = latestDiagnosticValue(key);
      return current && *current >= value;
    }, timeout);
  }

  void recreateMapper(const rclcpp::NodeOptions& options) {
    // Inject rebuild_only_map=false unless the caller explicitly overrides it,
    // so existing incremental-publication tests keep working without changes.
    // Also inject map_rebuild_min_interval_s=0.0 so back-to-back rebuild
    // requests in tests aren't delayed by the production throttle default.
    rclcpp::NodeOptions merged = options;
    bool has_rebuild_only = false;
    bool has_rebuild_interval = false;
    for (const auto& p : merged.parameter_overrides()) {
      if (p.get_name() == "rebuild_only_map") { has_rebuild_only = true; }
      else if (p.get_name() == "map_rebuild_min_interval_s") { has_rebuild_interval = true; }
    }
    auto overrides = merged.parameter_overrides();
    if (!has_rebuild_only) overrides.emplace_back("rebuild_only_map", false);
    if (!has_rebuild_interval) overrides.emplace_back("map_rebuild_min_interval_s", 0.0);
    merged.parameter_overrides(overrides);
    mapper_.reset();
    mapper_ = std::make_shared<MapperNode>(merged);
    spinFlush(mapper_, helper_, 200ms);
  }

  bool waitForPublishedMap(std::chrono::milliseconds timeout = 8000ms) {
    return spinUntil2(mapper_, helper_, [this] {
      return coherentPublishedMap().has_value();
    }, timeout);
  }

  std::optional<nav_msgs::msg::OccupancyGrid> mapForRevision(
      const orb_slam3_msgs::msg::MapRevision& revision) const {
    const auto map = std::find_if(maps_.begin(), maps_.end(), [&revision](const auto& candidate) {
      return candidate.header.stamp == revision.header.stamp;
    });
    if (map == maps_.end()) return std::nullopt;
    return *map;
  }

  std::optional<nav_msgs::msg::OccupancyGrid> coherentPublishedMap() const {
    for (auto revision = map_revisions_.rbegin(); revision != map_revisions_.rend(); ++revision) {
      if (revision->state != orb_slam3_msgs::msg::MapRevision::PUBLISHED) continue;
      const auto map = mapForRevision(*revision);
      if (map) return map;
    }
    return std::nullopt;
  }

  std::string publicationHistory() const {
    std::ostringstream out;
    out << "maps=";
    for (const auto& stamp : map_stamps_) out << stamp.sec << '.' << stamp.nanosec << ',';
    out << " revisions=";
    for (const auto& revision : map_revisions_) {
      out << static_cast<int>(revision.state) << '@' << revision.header.stamp.sec <<
        '.' << revision.header.stamp.nanosec << ',';
    }
    return out.str();
  }

  std::string endpointCounts() const {
    std::ostringstream out;
    out << " endpoints(odom=" << odom_pub_->get_subscription_count()
        << ",tracked=" << tracked_pub_->get_subscription_count()
        << ",scan=" << scan_pub_->get_subscription_count()
        << ",graph=" << graph_pub_->get_subscription_count()
        << ",event=" << event_pub_->get_subscription_count()
        << ",map=" << mapper_->count_subscribers("/orb_lidar/map")
        << ",revision=" << mapper_->count_subscribers("/orb_lidar/map_revision") << ')';
    return out.str();
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
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;

  std::shared_ptr<MapperNode> mapper_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr last_map_;
  orb_slam3_msgs::msg::MapRevision::ConstSharedPtr last_map_revision_;
  orb_slam3_msgs::msg::MapRevision::ConstSharedPtr last_published_map_revision_;
  visualization_msgs::msg::Marker::ConstSharedPtr last_prov_;
  nav_msgs::msg::Path::ConstSharedPtr last_wheel_path_;
  nav_msgs::msg::Path::ConstSharedPtr last_corr_path_;
  std::vector<diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr> diagnostics_;
  std::size_t map_rev_count_{0};
  std::vector<builtin_interfaces::msg::Time> map_stamps_;
  std::vector<builtin_interfaces::msg::Time> map_revision_stamps_;
  std::vector<nav_msgs::msg::OccupancyGrid> maps_;
  std::vector<orb_slam3_msgs::msg::MapRevision> map_revisions_;
  std::string endpoint_counts_before_sweep_;
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

TEST_F(MapperNodeTest, WheelPathPublisherIsDisabledByDefault) {
  EXPECT_TRUE(mapper_->get_publishers_info_by_topic("/orb_lidar/wheel_path").empty());
}

TEST_F(MapperNodeTest, OptionalWheelPathPublisherUsesBoundedHistory) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("publish_wheel_path", true),
      rclcpp::Parameter("wheel_path_max_points", 2)});
  recreateMapper(options);

  publishWheelBurst(1, 3);
  // Wait for the *final* burst sample to be reflected rather than merely for
  // poses.size() == 2, which can also be true transiently (e.g. right after
  // sample 2 lands, before sample 3 has been processed and evicts sample 1).
  ASSERT_TRUE(spinUntil2(mapper_, helper_, [this] {
    return last_wheel_path_ && !last_wheel_path_->poses.empty() &&
           last_wheel_path_->poses.back().header.stamp.sec == 3;
  }));
  ASSERT_TRUE(last_wheel_path_);
  ASSERT_EQ(last_wheel_path_->poses.size(), 2u);
  EXPECT_EQ(last_wheel_path_->poses[0].header.stamp.sec, 2);
  EXPECT_EQ(last_wheel_path_->poses[1].header.stamp.sec, 3);
}

TEST_F(MapperNodeTest, PublishedMapAndRevisionUseOnePairingStamp) {
  publishStraightSweepWithTwoVisualAnchors();
  ASSERT_TRUE(waitForPublishedMap()) << endpoint_counts_before_sweep_ << ' ' << publicationHistory();
  ASSERT_TRUE(last_map_);
  ASSERT_TRUE(last_published_map_revision_);
  EXPECT_EQ(last_published_map_revision_->state,
            orb_slam3_msgs::msg::MapRevision::PUBLISHED);
  const auto paired_map = mapForRevision(*last_published_map_revision_);
  ASSERT_TRUE(paired_map.has_value()) << publicationHistory();
  EXPECT_EQ(paired_map->header.stamp, last_published_map_revision_->header.stamp);
}

TEST_F(MapperNodeTest, StationaryFullSweepCommits) {
  publishPreparedSweep(graph_rev_base_ + 1u, 0.0, 0.0, 0.0);
  ASSERT_TRUE(waitForCommittedScanCount(1));
}

TEST_F(MapperNodeTest, StraightFullSweepCommits) {
  publishPreparedSweep(graph_rev_base_ + 1u, 0.0, 0.0);
  ASSERT_TRUE(waitForCommittedScanCount(1));
}

TEST_F(MapperNodeTest, MaximumYawExcursionAtLimitCommits) {
  publishPreparedSweep(graph_rev_base_ + 1u, 0.005, 0.005);
  ASSERT_TRUE(waitForCommittedScanCount(1));
}

TEST_F(MapperNodeTest, YawExcursionAboveLimitRejectsWithoutMapRevisionChange) {
  publishPreparedSweep(graph_rev_base_ + 1u, 0.005001, 0.005001);
  ASSERT_TRUE(waitForDiagnosticAtLeast("scans_turn_rejected", 1));
  EXPECT_EQ(map_rev_count_, 0u);
}

TEST_F(MapperNodeTest, TurnThenReturnExcursionRejectsWithoutMapRevisionChange) {
  publishPreparedSweep(graph_rev_base_ + 1u, 0.006, 0.0);
  ASSERT_TRUE(waitForDiagnosticAtLeast("scans_turn_rejected", 1));
  EXPECT_EQ(map_rev_count_, 0u);
}

TEST_F(MapperNodeTest, MissingFinalWheelCoverageWaitsThenCommits) {
  const uint64_t revision = graph_rev_base_ + 1u;
  publishSweepWheelCoverage(0.0, 0.0, false);
  publishVisualBracket(revision);
  scan_pub_->publish(sweep(2'000'000'000LL, 0.05F, {2.0F, 2.0F, 2.0F}));
  const std::size_t revisions_before = map_rev_count_;
  EXPECT_FALSE(spinUntil2(mapper_, helper_, [this, revisions_before] {
    return map_rev_count_ != revisions_before;
  }, 300ms));

  odom_pub_->publish(odom_ns(2'100'000'000LL, 0.02));
  odom_pub_->publish(odom_ns(2'150'000'000LL, 0.03));
  ASSERT_TRUE(waitForCommittedScanCount(1));
}

TEST_F(MapperNodeTest, MissingVisualEndAnchorWaitsThenCommits) {
  const uint64_t revision = graph_rev_base_ + 1u;
  publishSweepWheelCoverage();
  publishVisualBracket(revision, false);
  scan_pub_->publish(sweep(2'000'000'000LL, 0.05F, {2.0F, 2.0F, 2.0F}));
  const std::size_t revisions_before = map_rev_count_;
  EXPECT_FALSE(spinUntil2(mapper_, helper_, [this, revisions_before] {
    return map_rev_count_ != revisions_before;
  }, 300ms));

  tracked_pub_->publish(tracked_ns(2'150'000'000LL,
      orb_slam3_msgs::msg::TrackedFrame::OK, true, 0.03, 1, 1, revision));
  ASSERT_TRUE(waitForCommittedScanCount(1));
}

TEST_F(MapperNodeTest, VisualTimeoutIncrementsNoAnchorAndTimeoutDropped) {
  publishSweepWheelCoverage();
  scan_pub_->publish(sweep(2'000'000'000LL, 0.05F, {2.0F, 2.0F, 2.0F}));
  tracked_pub_->publish(tracked_ns(4'200'000'000LL,
      orb_slam3_msgs::msg::TrackedFrame::OK, true, 0.0, 1, 1, graph_rev_base_ + 1u));
  ASSERT_TRUE(waitForDiagnosticAtLeast("scans_no_anchor", 1));
  ASSERT_TRUE(waitForDiagnosticAtLeast("scans_timeout_dropped", 1));
}

TEST_F(MapperNodeTest, PendingQueueLimitDropsOldestAndRemainsBounded) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("pending_scan_limit", 2)});
  recreateMapper(options);
  publishSweepWheelCoverage();
  scan_pub_->publish(sweep(2'000'000'000LL, 0.05F, {2.0F, 2.0F, 2.0F}));
  scan_pub_->publish(sweep(2'050'000'000LL, 0.025F, {2.0F, 2.0F, 2.0F}));
  scan_pub_->publish(sweep(2'075'000'000LL, 0.0125F, {2.0F, 2.0F, 2.0F}));
  ASSERT_TRUE(waitForDiagnosticAtLeast("scans_timeout_dropped", 1));

  const uint64_t revision = graph_rev_base_ + 1u;
  publishVisualBracket(revision);
  ASSERT_TRUE(waitForCommittedScanCount(2));
  EXPECT_EQ(latestDiagnosticValue("scans_timeout_dropped"), std::optional<uint64_t>(1));

  graph_pub_->publish(make_graph(4, revision + 1u, 1));
  ASSERT_TRUE(spinUntil2(mapper_, helper_, [this] {
    return last_corr_path_ && last_corr_path_->poses.size() == 2u;
  }));
  const std::set<std::int64_t> retained_stamps{
    rclcpp::Time(last_corr_path_->poses[0].header.stamp).nanoseconds(),
    rclcpp::Time(last_corr_path_->poses[1].header.stamp).nanoseconds()};
  EXPECT_EQ(retained_stamps,
            (std::set<std::int64_t>{2'050'000'000LL, 2'075'000'000LL}));
  EXPECT_EQ(retained_stamps.count(2'000'000'000LL), 0u);
  ASSERT_TRUE(last_published_map_revision_);
  EXPECT_EQ(last_published_map_revision_->committed_scan_count, 2u);
  EXPECT_EQ(last_corr_path_->poses.size(), 2u)
      << "queue must retain exactly two scans with no third pending item";
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

TEST_F(MapperNodeTest, LostEventBeforeTrackedLostFrameKeepsLossSweepUncommittedUntilRecovery) {
  const uint64_t revision = graph_rev_base_ + 1u;
  event_pub_->publish(make_event(2, orb_slam3_msgs::msg::TrackingEvent::LOST, revision));
  spinFlush(mapper_, helper_, 100ms);
  publishSweepWheelCoverage();
  scan_pub_->publish(sweep(2'000'000'000LL, 0.05F, {2.0F, 2.0F, 2.0F}));
  tracked_pub_->publish(tracked_ns(2'150'000'000LL,
      orb_slam3_msgs::msg::TrackedFrame::LOST, false, 0.0, 1, 1, revision));

  EXPECT_FALSE(waitForCommittedScanCount(1, 500ms));
  EXPECT_EQ(map_rev_count_, 0u);
}

TEST_F(MapperNodeTest, TrackedLostFrameBeforeLostEventKeepsLossSweepUncommittedUntilRecovery) {
  const uint64_t revision = graph_rev_base_ + 1u;
  tracked_pub_->publish(tracked_ns(1'950'000'000LL,
      orb_slam3_msgs::msg::TrackedFrame::LOST, false, 0.0, 1, 1, revision));
  spinFlush(mapper_, helper_, 100ms);
  event_pub_->publish(make_event(2, orb_slam3_msgs::msg::TrackingEvent::LOST, revision));
  publishSweepWheelCoverage();
  scan_pub_->publish(sweep(2'000'000'000LL, 0.05F, {2.0F, 2.0F, 2.0F}));

  EXPECT_FALSE(waitForCommittedScanCount(1, 500ms));
  EXPECT_EQ(map_rev_count_, 0u);
}

TEST_F(MapperNodeTest, RecoveryKeepsExactMapUntilCorrectedRebuildPublishes) {
  const uint64_t revision = graph_rev_base_ + 1u;
  const uint64_t corrected_revision = revision + 1u;
  publishWheelBurst(0, 5);
  spinFlush(mapper_, helper_, 200ms);
  tracked_pub_->publish(make_tracked(2, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 0.0, 1, 1, revision));
  graph_pub_->publish(make_graph(2, revision, 1));
  spinFlush(mapper_, helper_, 200ms);
  scan_pub_->publish(make_scan(2, 2.0f));
  ASSERT_TRUE(waitForCommittedScanCount(1));
  ASSERT_TRUE(last_map_);
  ASSERT_TRUE(last_published_map_revision_);
  const auto old_map = mapForRevision(*last_published_map_revision_);
  ASSERT_TRUE(old_map.has_value());
  EXPECT_EQ(old_map->header.stamp, last_published_map_revision_->header.stamp);
  const auto old_data = old_map->data;
  const uint64_t old_map_revision = last_published_map_revision_->map_revision;

  event_pub_->publish(make_event(3, orb_slam3_msgs::msg::TrackingEvent::LOST, revision));
  tracked_pub_->publish(make_tracked(3, orb_slam3_msgs::msg::TrackedFrame::LOST,
                                     false, 0.0, 1, 1, revision));
  event_pub_->publish(make_event(8, orb_slam3_msgs::msg::TrackingEvent::RELOCALIZED,
                                 corrected_revision));
  tracked_pub_->publish(make_tracked(9, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 1.0, 1, 1, corrected_revision));
  spinFlush(mapper_, helper_, 300ms);

  ASSERT_TRUE(last_published_map_revision_);
  const auto still_paired_old_map = mapForRevision(*last_published_map_revision_);
  ASSERT_TRUE(still_paired_old_map.has_value());
  EXPECT_EQ(still_paired_old_map->header.stamp,
            last_published_map_revision_->header.stamp);
  EXPECT_EQ(still_paired_old_map->data, old_data);
  EXPECT_EQ(last_published_map_revision_->map_revision, old_map_revision);

  graph_pub_->publish(make_graph(9, corrected_revision, 1, 1.0));
  ASSERT_TRUE(spinUntil2(mapper_, helper_, [this, old_map_revision] {
    return last_published_map_revision_ &&
      last_published_map_revision_->map_revision > old_map_revision;
  }, 12000ms));
}

TEST_F(MapperNodeTest, DestroyingNodeWithQueuedScansAndRebuildWorkIsSafe) {
  const uint64_t revision = graph_rev_base_ + 1u;
  mapper_node_test_hooks::armPublishBarrier();
  publishPreparedSweep(revision);
  spinFlush(mapper_, helper_, 200ms);
  ASSERT_TRUE(mapper_node_test_hooks::waitForPublishBarrier(4000ms));

  scan_pub_->publish(sweep(2'200'000'000LL, 0.05F, {2.0F, 2.0F, 2.0F}));
  spinFlush(mapper_, helper_, 100ms);

  std::atomic_bool destroy_completed{false};
  auto node = std::move(mapper_);
  std::thread destroyer([&destroy_completed, node = std::move(node)]() mutable {
    node.reset();
    destroy_completed.store(true);
  });
  ASSERT_TRUE(mapper_node_test_hooks::waitForDestructorEntry(1000ms));
  EXPECT_FALSE(destroy_completed.load());

  mapper_node_test_hooks::releasePublishBarrier();
  destroyer.join();
  EXPECT_TRUE(mapper_node_test_hooks::publishCompleted());
  EXPECT_TRUE(destroy_completed.load());
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
  EXPECT_EQ(map_i[0].qos_profile().depth(), 10u);

  ASSERT_EQ(rev_i.size(), 1u);
  EXPECT_EQ(rev_i[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(rev_i[0].qos_profile().durability(),  rclcpp::DurabilityPolicy::TransientLocal);
  EXPECT_EQ(rev_i[0].qos_profile().depth(), 10u);
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
// Nearest-anchor snap: a tracked frame whose wheel interpolation fails at
// ingestion (e.g. live sim-time skew — wheel odometry has not arrived yet
// when the tracked frame is processed) must still be anchored and eventually
// produce a committed scan, instead of being silently dropped.
// ---------------------------------------------------------------------------
TEST_F(MapperNodeTest, TrackedFrameWithoutWheelDataAtIngestionStillProducesCommittedScan) {
  const uint64_t rev1 = graph_rev_base_ + 1u;

  // No wheel odometry has been published yet — wheel_buf_ is empty, so
  // interpolation at the tracked-frame stamp must fail. Under the old
  // behavior this dropped the anchor entirely; the fix must anchor it anyway.
  tracked_pub_->publish(make_tracked(2, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 0.0, 1, 1, rev1));
  spinFlush(mapper_, helper_, 200ms);

  graph_pub_->publish(make_graph(2, rev1, 1));
  spinFlush(mapper_, helper_, 300ms);

  // Wheel odometry arrives afterward, covering the scan's own timestamp
  // (needed by the unrelated, ray-level ScanDeskewer — not by the anchor).
  publishWheelBurst(0, 5);
  spinFlush(mapper_, helper_, 300ms);

  const std::size_t rev_count_before = map_rev_count_;
  scan_pub_->publish(make_scan(2, 2.0f));

  bool ok = spinUntil2(mapper_, helper_,
    [this, rev_count_before] {
      return map_rev_count_ > rev_count_before &&
             last_map_revision_ &&
             last_map_revision_->committed_scan_count >= 1u;
    });

  ASSERT_TRUE(ok) << "Nearest-anchor snap should still allow the scan to "
                     "commit even though wheel odometry was unavailable when "
                     "the tracked frame arrived";
}

// ---------------------------------------------------------------------------
// Test 7: Default parameters
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// rebuild_only_map: in default mode, no incremental map is published until a
// graph snapshot triggers a full rebuild. This eliminates ghost walls from
// scans placed at pre-correction (drifted) poses between loop closures.
// ---------------------------------------------------------------------------
TEST_F(MapperNodeTest, RebuildOnlyMapSkipsIncrementalAndPublishesOnFullRebuild) {
  // Create mapper with production default: rebuild_only_map=true.
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("rebuild_only_map", true)});
  recreateMapper(options);

  publishWheelBurst(0, 5);
  spinFlush(mapper_, helper_, 300ms);

  tracked_pub_->publish(make_tracked(2, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 0.0, 1, 1, 1));
  spinFlush(mapper_, helper_, 200ms);

  // Graph snapshot at revision 1 — commits the keyframe but also requests a
  // full rebuild (which requires at least one committed scan in the archive).
  const uint64_t revision = graph_rev_base_ + 1u;
  graph_pub_->publish(make_graph(2, revision, 1));
  spinFlush(mapper_, helper_, 300ms);

  // Scan committed at t=2. In rebuild-only mode, appendCommitted is skipped,
  // so no incremental map. A subsequent graph snapshot triggers the rebuild.
  scan_pub_->publish(make_scan(2, 2.0f));
  spinFlush(mapper_, helper_, 500ms);

  // No map yet — incremental was skipped.
  EXPECT_EQ(last_map_, nullptr)
      << "rebuild_only_map=true should not produce an incremental map";

  // A new graph snapshot triggers the full rebuild, which includes the scan.
  graph_pub_->publish(make_graph(3, revision + 1u, 1));
  ASSERT_TRUE(spinUntil2(mapper_, helper_, [this] {
    return last_published_map_revision_ &&
           last_published_map_revision_->committed_scan_count >= 1u;
  }));
  ASSERT_NE(last_map_, nullptr);
  EXPECT_EQ(last_map_->header.frame_id, "orb_map");
  // The rebuild produces occupied + free cells from the corrected pose.
  bool has_occ = false, has_free = false;
  for (int8_t c : last_map_->data) {
    if (c > 50)             has_occ  = true;
    if (c >= 0 && c <= 30)  has_free = true;
  }
  EXPECT_TRUE(has_occ);
  EXPECT_TRUE(has_free);
}

TEST_F(MapperNodeTest, RebuildOnlyMapDisabledAllowsIncrementalPublish) {
  // Opt out: rebuild_only_map=false restores incremental behaviour.
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("rebuild_only_map", false)});
  recreateMapper(options);

  publishWheelBurst(0, 5);
  spinFlush(mapper_, helper_, 300ms);

  tracked_pub_->publish(make_tracked(2, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 0.0, 1, 1, 1));
  spinFlush(mapper_, helper_, 200ms);

  const uint64_t revision = graph_rev_base_ + 1u;
  graph_pub_->publish(make_graph(2, revision, 1));
  spinFlush(mapper_, helper_, 300ms);

  // Scan should produce an incremental map publication immediately.
  scan_pub_->publish(make_scan(2, 2.0f));
  ASSERT_TRUE(spinUntil2(mapper_, helper_, [this] {
    return last_published_map_revision_ &&
           last_published_map_revision_->committed_scan_count >= 1u;
  }));
  ASSERT_NE(last_map_, nullptr);
}

TEST_F(MapperNodeTest, PureAdditionGraphSnapshotSkipsFullRebuild) {
  // A graph snapshot that only advances the revision without shifting any
  // existing keyframe pose ("pure addition") must NOT trigger a new full
  // rebuild. The map is already consistent; a rebuild would be pure waste.
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("rebuild_only_map", true)});
  recreateMapper(options);

  // Prime state: wheel odom + tracked frame + a scan, so at least one scan
  // can be committed on the first graph snapshot.
  publishWheelBurst(0, 5);
  spinFlush(mapper_, helper_, 300ms);
  tracked_pub_->publish(make_tracked(2, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 0.0, 1, 1, 1));
  spinFlush(mapper_, helper_, 200ms);
  scan_pub_->publish(make_scan(2, 2.0f));
  spinFlush(mapper_, helper_, 300ms);

  // Graph #1: first snapshot always takes the correction path (pose cache
  // empty) and does a full rebuild.
  const uint64_t rev1 = graph_rev_base_ + 1u;
  graph_pub_->publish(make_graph(3, rev1, 1, 0.0));
  ASSERT_TRUE(spinUntil2(mapper_, helper_, [this] {
    return last_published_map_revision_ &&
           last_published_map_revision_->committed_scan_count >= 1u;
  }));
  const std::uint64_t first_map_revision = last_published_map_revision_->map_revision;
  // Wait for the first rebuild's trailing IDLE emit so it doesn't leak into
  // the count we compare against below.
  spinFlush(mapper_, helper_, 300ms);
  const std::size_t published_count_before = std::count_if(
      map_revisions_.begin(), map_revisions_.end(),
      [](const orb_slam3_msgs::msg::MapRevision& m) {
        return m.state == orb_slam3_msgs::msg::MapRevision::PUBLISHED;
      });

  // Graph #2: same keyframe id + same pose, just a bumped graph revision.
  // Fast-path detection must see zero pose shift and skip requestRebuild.
  graph_pub_->publish(make_graph(4, rev1 + 1u, 1, 0.0));
  spinFlush(mapper_, helper_, 800ms);

  EXPECT_EQ(last_published_map_revision_->map_revision, first_map_revision)
      << "Fast-path (no pose shift) snapshot must not trigger a new full rebuild";
  const std::size_t published_count_after = std::count_if(
      map_revisions_.begin(), map_revisions_.end(),
      [](const orb_slam3_msgs::msg::MapRevision& m) {
        return m.state == orb_slam3_msgs::msg::MapRevision::PUBLISHED;
      });
  EXPECT_EQ(published_count_after, published_count_before)
      << "No new PUBLISHED MapRevision should follow a no-op fast-path snapshot";
}

TEST_F(MapperNodeTest, PoseShiftGraphSnapshotTriggersFullRebuild) {
  // The counterpart of the fast-path test: if an existing keyframe pose
  // shifts (loop closure / GBA correction), the mapper must take the
  // correction path and produce a new full rebuild.
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("rebuild_only_map", true)});
  recreateMapper(options);
  publishWheelBurst(0, 5);
  spinFlush(mapper_, helper_, 300ms);
  tracked_pub_->publish(make_tracked(2, orb_slam3_msgs::msg::TrackedFrame::OK,
                                     true, 0.0, 1, 1, 1));
  spinFlush(mapper_, helper_, 200ms);
  scan_pub_->publish(make_scan(2, 2.0f));
  spinFlush(mapper_, helper_, 300ms);

  const uint64_t rev1 = graph_rev_base_ + 1u;
  graph_pub_->publish(make_graph(3, rev1, 1, 0.0));
  ASSERT_TRUE(spinUntil2(mapper_, helper_, [this] {
    return last_published_map_revision_ &&
           last_published_map_revision_->committed_scan_count >= 1u;
  }));
  const std::uint64_t first_map_revision = last_published_map_revision_->map_revision;

  // Graph #2: same keyframe id but SHIFTED map pose (kf_x moved from 0 to 1).
  // The correction path must fire a full rebuild → new map_revision.
  graph_pub_->publish(make_graph(4, rev1 + 1u, 1, 1.0));
  ASSERT_TRUE(spinUntil2(mapper_, helper_, [this, first_map_revision] {
    return last_published_map_revision_ &&
           last_published_map_revision_->map_revision > first_map_revision;
  }));
}

TEST_F(MapperNodeTest, DefaultParametersAreDeclaredCorrectly) {
  // Create with true production defaults (no test overrides).
  auto prod_mapper = std::make_shared<MapperNode>();
  EXPECT_EQ(prod_mapper->get_parameter("odom_topic").as_string(),    "/odom_wheel");
  EXPECT_EQ(prod_mapper->get_parameter("scan_topic").as_string(),    "/scan_origin");
  EXPECT_EQ(prod_mapper->get_parameter("tracked_frame_topic").as_string(),
            "/orb_slam3/tracked_frame");
  EXPECT_EQ(prod_mapper->get_parameter("graph_snapshot_topic").as_string(),
            "/orb_slam3/graph_snapshot");
  EXPECT_EQ(prod_mapper->get_parameter("tracking_event_topic").as_string(),
            "/orb_slam3/events");
  EXPECT_EQ(prod_mapper->get_parameter("map_topic").as_string(),     "/orb_lidar/map");
  EXPECT_EQ(prod_mapper->get_parameter("map_frame").as_string(),     "orb_map");
  EXPECT_EQ(prod_mapper->get_parameter("base_frame").as_string(),    "base_link");
  EXPECT_NEAR(prod_mapper->get_parameter("wheel_retention_s").as_double(),  300.0, 1e-9);
  EXPECT_NEAR(prod_mapper->get_parameter("wheel_max_gap_ms").as_double(),   100.0, 1e-9);
  EXPECT_EQ(prod_mapper->get_parameter("imu_topic").as_string(), "/imu");
  EXPECT_TRUE(prod_mapper->get_parameter("enable_imu_deskew").as_bool());
  EXPECT_NEAR(prod_mapper->get_parameter("imu_retention_s").as_double(), 300.0, 1e-9);
  EXPECT_NEAR(prod_mapper->get_parameter("imu_max_gap_ms").as_double(), 20.0, 1e-9);
  EXPECT_NEAR(prod_mapper->get_parameter("resolution_m").as_double(),         0.05, 1e-9);
  EXPECT_NEAR(prod_mapper->get_parameter("usable_range_m").as_double(),       20.0, 1e-9);
  EXPECT_NEAR(prod_mapper->get_parameter("hit_range_max_m").as_double(), 10.0, 1e-9);
  EXPECT_NEAR(prod_mapper->get_parameter("hit_log_odds").as_double(), 0.55, 1e-6);
  EXPECT_NEAR(prod_mapper->get_parameter("miss_log_odds").as_double(), -0.50, 1e-6);
  EXPECT_NEAR(prod_mapper->get_parameter("max_roll_pitch_deg").as_double(),   10.0, 1e-9);
  EXPECT_NEAR(prod_mapper->get_parameter("max_height_delta_m").as_double(),   0.15, 1e-9);
  EXPECT_NEAR(prod_mapper->get_parameter("max_scan_yaw_change_rad").as_double(), 0.005, 1e-12);
  EXPECT_NEAR(prod_mapper->get_parameter("visual_anchor_max_gap_ms").as_double(), 200.0, 1e-9);
  EXPECT_NEAR(prod_mapper->get_parameter("pending_scan_timeout_s").as_double(), 2.0, 1e-9);
  EXPECT_EQ(prod_mapper->get_parameter("pending_scan_limit").as_int(), 200);
  EXPECT_TRUE(prod_mapper->get_parameter("rebuild_only_map").as_bool());
  EXPECT_NEAR(prod_mapper->get_parameter("map_rebuild_min_interval_s").as_double(), 5.0, 1e-9);
}

TEST_F(MapperNodeTest, ScanGateParametersRejectInvalidBoundaries) {
  const auto rejects = [](const rclcpp::Parameter& parameter) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({parameter});
    EXPECT_THROW((void)std::make_shared<MapperNode>(options), std::invalid_argument);
  };

  rejects(rclcpp::Parameter("max_scan_yaw_change_rad", -0.000001));
  rejects(rclcpp::Parameter("visual_anchor_max_gap_ms", -0.000001));
  rejects(rclcpp::Parameter("pending_scan_timeout_s", -0.000001));
  rejects(rclcpp::Parameter("pending_scan_limit", 0));
  rejects(rclcpp::Parameter("pending_scan_limit", -1));
  rejects(rclcpp::Parameter("max_scan_yaw_change_rad",
                            std::numeric_limits<double>::quiet_NaN()));
  rejects(rclcpp::Parameter("max_scan_yaw_change_rad",
                            std::numeric_limits<double>::infinity()));
  rejects(rclcpp::Parameter("visual_anchor_max_gap_ms",
                            std::numeric_limits<double>::quiet_NaN()));
  rejects(rclcpp::Parameter("visual_anchor_max_gap_ms",
                            std::numeric_limits<double>::infinity()));
  rejects(rclcpp::Parameter("pending_scan_timeout_s",
                            std::numeric_limits<double>::quiet_NaN()));
  rejects(rclcpp::Parameter("pending_scan_timeout_s",
                            std::numeric_limits<double>::infinity()));
}

}  // namespace
}  // namespace orb_lidar_mapper
