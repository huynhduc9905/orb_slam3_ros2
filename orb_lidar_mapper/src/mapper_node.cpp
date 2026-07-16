#include "orb_lidar_mapper/mapper_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <stdexcept>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace orb_lidar_mapper {

namespace {

int64_t toNs(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<int64_t>(stamp.nanosec);
}

std::optional<int64_t> scanEndStampNs(const sensor_msgs::msg::LaserScan& scan) {
  const int64_t start_ns = toNs(scan.header.stamp);
  if (!std::isfinite(scan.time_increment) || scan.time_increment < 0.0F) {
    return std::nullopt;
  }
  if (scan.ranges.empty()) {
    return start_ns;
  }
  const long double offset_ns = static_cast<long double>(scan.ranges.size() - 1U) *
    static_cast<long double>(scan.time_increment) * 1'000'000'000.0L;
  if (!std::isfinite(offset_ns)) {
    return std::nullopt;
  }
  const long double end_ns = static_cast<long double>(start_ns) + std::round(offset_ns);
  if (end_ns < static_cast<long double>(std::numeric_limits<int64_t>::min()) ||
      end_ns > static_cast<long double>(std::numeric_limits<int64_t>::max())) {
    return std::nullopt;
  }
  return static_cast<int64_t>(end_ns);
}

bool advancedBeyond(
  const std::optional<int64_t>& latest_ns, int64_t target_ns, int64_t timeout_ns) {
  if (!latest_ns || *latest_ns < target_ns) {
    return false;
  }
  return static_cast<std::uint64_t>(*latest_ns) - static_cast<std::uint64_t>(target_ns) >
    static_cast<std::uint64_t>(timeout_ns);
}

double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Pose2 poseFromMsg(const geometry_msgs::msg::Pose& p) {
  return {p.position.x, p.position.y, yawFromQuat(p.orientation)};
}

Pose2 poseFromTransform(const geometry_msgs::msg::Transform& t) {
  geometry_msgs::msg::Quaternion q;
  q.x = t.rotation.x; q.y = t.rotation.y;
  q.z = t.rotation.z; q.w = t.rotation.w;
  return {t.translation.x, t.translation.y, yawFromQuat(q)};
}

TrackingState fromRosState(uint8_t state) {
  switch (state) {
    case orb_slam3_msgs::msg::TrackedFrame::NOT_INITIALIZED: return TrackingState::kNotInitialized;
    case orb_slam3_msgs::msg::TrackedFrame::OK:              return TrackingState::kOk;
    case orb_slam3_msgs::msg::TrackedFrame::RECENTLY_LOST:   return TrackingState::kRecentlyLost;
    case orb_slam3_msgs::msg::TrackedFrame::LOST:            return TrackingState::kLost;
    default:                                                  return TrackingState::kNoImages;
  }
}

// Transform a local-frame point into the map frame via pose.
Point2 transformLocal(const Pose2& pose, const Point2& p) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y};
}

}  // namespace

namespace mapper_node_test_hooks {
namespace {

struct PublishBarrier {
  std::mutex mutex;
  std::condition_variable cv;
  bool armed{false};
  bool entered{false};
  bool released{false};
  bool completed{false};
  bool destructor_entered{false};
};

PublishBarrier& publishBarrier() {
  static PublishBarrier barrier;
  return barrier;
}

}  // namespace

void armPublishBarrier() {
  auto& barrier = publishBarrier();
  std::lock_guard<std::mutex> lock(barrier.mutex);
  barrier.armed = true;
  barrier.entered = false;
  barrier.released = false;
  barrier.completed = false;
  barrier.destructor_entered = false;
}

bool waitForPublishBarrier(std::chrono::milliseconds timeout) {
  auto& barrier = publishBarrier();
  std::unique_lock<std::mutex> lock(barrier.mutex);
  return barrier.cv.wait_for(lock, timeout, [&barrier] { return barrier.entered; });
}

bool waitForDestructorEntry(std::chrono::milliseconds timeout) {
  auto& barrier = publishBarrier();
  std::unique_lock<std::mutex> lock(barrier.mutex);
  return barrier.cv.wait_for(
    lock, timeout, [&barrier] { return barrier.destructor_entered; });
}

void releasePublishBarrier() {
  auto& barrier = publishBarrier();
  {
    std::lock_guard<std::mutex> lock(barrier.mutex);
    barrier.released = true;
  }
  barrier.cv.notify_all();
}

bool publishCompleted() {
  auto& barrier = publishBarrier();
  std::lock_guard<std::mutex> lock(barrier.mutex);
  return barrier.completed;
}

void resetPublishBarrier() {
  auto& barrier = publishBarrier();
  {
    std::lock_guard<std::mutex> lock(barrier.mutex);
    barrier.armed = false;
    barrier.entered = false;
    barrier.released = true;
    barrier.completed = false;
    barrier.destructor_entered = false;
  }
  barrier.cv.notify_all();
}

void beforePublish() {
  auto& barrier = publishBarrier();
  std::unique_lock<std::mutex> lock(barrier.mutex);
  if (!barrier.armed) return;
  barrier.entered = true;
  barrier.cv.notify_all();
  barrier.cv.wait(lock, [&barrier] { return barrier.released; });
}

void afterPublish() {
  auto& barrier = publishBarrier();
  std::lock_guard<std::mutex> lock(barrier.mutex);
  if (barrier.armed) barrier.completed = true;
}

void notifyDestructorEntry() {
  auto& barrier = publishBarrier();
  {
    std::lock_guard<std::mutex> lock(barrier.mutex);
    if (barrier.armed) barrier.destructor_entered = true;
  }
  barrier.cv.notify_all();
}

}  // namespace mapper_node_test_hooks

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

MapperNode::MapperNode(const rclcpp::NodeOptions& options)
: Node("orb_lidar_mapper", options),
  odom_topic_(declare_parameter("odom_topic", "/odom_wheel")),
  scan_topic_(declare_parameter("scan_topic", "/scan_origin")),
  tracked_frame_topic_(declare_parameter("tracked_frame_topic", "/orb_slam3/tracked_frame")),
  graph_snapshot_topic_(declare_parameter("graph_snapshot_topic", "/orb_slam3/graph_snapshot")),
  tracking_event_topic_(declare_parameter("tracking_event_topic", "/orb_slam3/events")),
  map_topic_(declare_parameter("map_topic", "/orb_lidar/map")),
  map_frame_(declare_parameter("map_frame", "orb_map")),
  base_frame_(declare_parameter("base_frame", "base_link")),
  wheel_retention_s_(declare_parameter("wheel_retention_s", 300.0)),
  wheel_max_gap_ms_(declare_parameter("wheel_max_gap_ms", 100.0)),
  resolution_m_(declare_parameter("resolution_m", 0.05)),
  usable_range_m_(declare_parameter("usable_range_m", 20.0)),
  max_roll_pitch_deg_(declare_parameter("max_roll_pitch_deg", 10.0)),
  max_height_delta_m_(declare_parameter("max_height_delta_m", 0.15)),
  max_scan_yaw_change_rad_(declare_parameter("max_scan_yaw_change_rad", 0.005)),
  visual_anchor_max_gap_ms_(declare_parameter("visual_anchor_max_gap_ms", 200.0)),
  pending_scan_timeout_s_(declare_parameter("pending_scan_timeout_s", 2.0)),
  pending_scan_limit_(declare_parameter("pending_scan_limit", 200)) {

  if (!std::isfinite(max_scan_yaw_change_rad_) ||
      !std::isfinite(visual_anchor_max_gap_ms_) ||
      !std::isfinite(pending_scan_timeout_s_) ||
      max_scan_yaw_change_rad_ < 0.0 || visual_anchor_max_gap_ms_ < 0.0 ||
      pending_scan_timeout_s_ < 0.0 || pending_scan_limit_ <= 0 ||
      visual_anchor_max_gap_ms_ >
        static_cast<double>(std::numeric_limits<int64_t>::max()) / 1e6 ||
      pending_scan_timeout_s_ >
        static_cast<double>(std::numeric_limits<int64_t>::max()) / 1e9) {
    throw std::invalid_argument("scan gating and pending-queue parameters are invalid");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  const int64_t retention_ns = static_cast<int64_t>(wheel_retention_s_ * 1e9);
  const int64_t max_gap_ns   = static_cast<int64_t>(wheel_max_gap_ms_ * 1e6);

  traj_ = std::make_unique<TrajectoryStore>(TrajectoryConfig{retention_ns, max_gap_ns});
  // Mirror wheel buffer used for deskewing — same parameters as TrajectoryStore's internal buffer.
  wheel_buf_ = std::make_unique<TimedPoseBuffer>(retention_ns, max_gap_ns);
  archive_ = std::make_shared<ScanArchive>();

  GridConfig grid_cfg;
  grid_cfg.resolution_m  = resolution_m_;
  grid_cfg.usable_range_m = usable_range_m_;

  rebuilder_ = std::make_unique<MapRebuilder>(
      grid_cfg,
      [this](std::shared_ptr<const MapSnapshot> snap, const RebuildStatus& status) {
        publishMapAndRevision(std::move(snap), status);
      });

  // QoS
  auto sensor_qos = rclcpp::SensorDataQoS();
  auto snapshot_qos = rclcpp::QoS(1).reliable().transient_local();
  auto map_history_qos = rclcpp::QoS(10).reliable().transient_local();
  auto reliable = rclcpp::QoS(100).reliable();
  auto reliable10 = rclcpp::QoS(10).reliable();

  // Publishers
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, map_history_qos);
  map_rev_pub_ = create_publisher<orb_slam3_msgs::msg::MapRevision>(
      "/orb_lidar/map_revision", map_history_qos);
  corrected_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/orb_lidar/corrected_path", snapshot_qos);
  corrected_path_rev_pub_ = create_publisher<orb_slam3_msgs::msg::RevisionedPath>(
      "/orb_lidar/corrected_path_revisioned", snapshot_qos);
  wheel_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/orb_lidar/wheel_path", reliable10);
  provisional_scan_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/orb_lidar/provisional_scan", reliable10);
  committed_scan_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/orb_lidar/committed_scan", reliable10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", reliable);

  // Subscriptions
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onOdom(msg); });

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, sensor_qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { onScan(msg); });

  tracked_sub_ = create_subscription<orb_slam3_msgs::msg::TrackedFrame>(
      tracked_frame_topic_, sensor_qos,
      [this](orb_slam3_msgs::msg::TrackedFrame::ConstSharedPtr msg) { onTrackedFrame(msg); });

  graph_sub_ = create_subscription<orb_slam3_msgs::msg::GraphSnapshot>(
      graph_snapshot_topic_, snapshot_qos,
      [this](orb_slam3_msgs::msg::GraphSnapshot::ConstSharedPtr msg) { onGraphSnapshot(msg); });

  event_sub_ = create_subscription<orb_slam3_msgs::msg::TrackingEvent>(
      tracking_event_topic_, reliable,
      [this](orb_slam3_msgs::msg::TrackingEvent::ConstSharedPtr msg) { onTrackingEvent(msg); });
}

MapperNode::~MapperNode() {
  // The rebuilder worker invokes a callback that captures this node. Stop and
  // join it before publishers and the other node members begin destruction.
  mapper_node_test_hooks::notifyDestructorEntry();
  rebuilder_.reset();
}

// ─────────────────────────────────────────────────────────────────────────────
// Odometry
// ─────────────────────────────────────────────────────────────────────────────

void MapperNode::onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  const int64_t stamp_ns = toNs(msg->header.stamp);
  const Pose2 pose = poseFromMsg(msg->pose.pose);
  const TimedPose2 sample{stamp_ns, pose};

  std::lock_guard<std::mutex> lock(mutex_);
  traj_->addWheel(sample);
  wheel_buf_->push(sample);

  // Wheel path
  geometry_msgs::msg::PoseStamped ps;
  ps.header = msg->header;
  ps.header.frame_id = map_frame_;
  ps.pose = msg->pose.pose;
  wheel_poses_.push_back(ps);

  nav_msgs::msg::Path path;
  path.header.stamp = msg->header.stamp;
  path.header.frame_id = map_frame_;
  path.poses = wheel_poses_;
  wheel_path_pub_->publish(path);

  processPendingScansLocked();
}

// ─────────────────────────────────────────────────────────────────────────────
// Tracked frame
// ─────────────────────────────────────────────────────────────────────────────

void MapperNode::onTrackedFrame(orb_slam3_msgs::msg::TrackedFrame::ConstSharedPtr msg) {
  const int64_t stamp_ns = toNs(msg->header.stamp);
  const TrackingState state = fromRosState(msg->tracking_state);

  FrameAnchor anchor;
  anchor.stamp_ns = stamp_ns;
  anchor.state = state;
  anchor.pose_valid = msg->pose_valid;
  anchor.map_id = msg->map_id;
  anchor.reference_keyframe_id = msg->reference_keyframe_id;
  anchor.graph_revision = msg->graph_revision;
  if (msg->pose_valid) {
    anchor.map_pose = poseFromMsg(msg->pose);
    anchor.reference_to_frame = poseFromTransform(msg->reference_to_frame);
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Populate wheel_pose when possible. If odometry for this sensor timestamp
  // has not arrived yet, TrajectoryStore::refreshWheelData() backfills it on a
  // later odometry callback; committed scans wait for that bridge instead of
  // permanently snapping to the visual frame timestamp.
  const auto wheel_at_frame = wheel_buf_->interpolate(stamp_ns);
  if (!wheel_at_frame) {
    ++wheel_interp_failures_;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "wheel interpolation unavailable at tracked-frame stamp %ld ns; "
        "deferring dependent committed scans until odometry backfill",
        static_cast<long>(stamp_ns));
    publishDiagnostics("WARN",
        "tracked frame waiting for wheel-odometry backfill");
  }
  anchor.wheel_pose = wheel_at_frame;
  traj_->addTrackedFrame(anchor);
  processPendingScansLocked();
}

// ─────────────────────────────────────────────────────────────────────────────
// Graph snapshot
// ─────────────────────────────────────────────────────────────────────────────

void MapperNode::onGraphSnapshot(orb_slam3_msgs::msg::GraphSnapshot::ConstSharedPtr msg) {
  GraphSnapshotValue snap;
  snap.graph_revision      = msg->revision;
  snap.active_map_id       = msg->active_map_id;
  snap.active_map_connected = msg->active_map_connected;
  for (const auto& kf : msg->keyframes) {
    KeyframeValue kv;
    kv.keyframe_id = kf.id;
    kv.map_id      = kf.map_id;
    kv.map_pose    = poseFromMsg(kf.pose);
    snap.keyframes.push_back(kv);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (snap.graph_revision <= last_graph_revision_) return;
  last_graph_revision_ = snap.graph_revision;

  traj_->applyGraphSnapshot(snap);

  // After graph correction, rebuild from the corrected trajectory.
  auto revision = traj_->snapshot();

  // Build a filtered archive and trajectory containing only scan_ids present
  // in BOTH the archive and the trajectory. validate() requires exact parity.
  std::unordered_set<uint64_t> archived_ids;
  for (const auto& arc : archive_->scans) archived_ids.insert(arc.scan_id);

  std::unordered_set<uint64_t> trajectory_ids;
  for (const auto& sp : revision->scans) trajectory_ids.insert(sp.scan_id);

  // Common ids: present in both.
  std::unordered_set<uint64_t> common_ids;
  for (auto id : archived_ids) {
    if (trajectory_ids.count(id)) common_ids.insert(id);
  }

  auto filtered_arc = std::make_shared<ScanArchive>();
  for (const auto& arc : archive_->scans) {
    if (common_ids.count(arc.scan_id)) filtered_arc->scans.push_back(arc);
  }

  auto filtered_traj = std::make_shared<TrajectoryRevision>();
  filtered_traj->graph_revision = revision->graph_revision;
  filtered_traj->frames = revision->frames;
  for (const auto& sp : revision->scans) {
    if (common_ids.count(sp.scan_id)) filtered_traj->scans.push_back(sp);
  }

  // Count committed scans in the filtered trajectory (rebuild needs at least one).
  std::size_t committed_in_traj = 0;
  for (const auto& sp : filtered_traj->scans) {
    if (sp.committed) ++committed_in_traj;
  }

  // Disconnected atlas or empty committed set: freeze last good map.
  // Still update corrected path; do not rebuild or append into the live grid.
  if (!snap.active_map_connected || committed_in_traj == 0 || filtered_arc->scans.empty()) {
    publishCorrectedPath(revision);
    if (!snap.active_map_connected) {
      publishDiagnostics("WARN", "graph snapshot disconnected: keeping last published map frozen");
    }
    processPendingScansLocked();
    return;
  }

  // Feed newly-committed scans incrementally as well.
  for (const auto& sp : filtered_traj->scans) {
    if (sp.committed && committed_scan_ids_.find(sp.scan_id) == committed_scan_ids_.end()) {
      for (const auto& arc : filtered_arc->scans) {
        if (arc.scan_id == sp.scan_id) {
          rebuilder_->appendCommitted(arc, sp, snap.graph_revision);
          committed_scan_ids_.insert(sp.scan_id);
          break;
        }
      }
    }
  }

  rebuilder_->requestRebuild(filtered_traj, filtered_arc);

  publishCorrectedPath(revision);

  // Recovery: clear provisional state after successful graph update.
  if (was_lost_) {
    was_lost_ = false;
    deleteProvisionalMarkers();
  }
  processPendingScansLocked();
}

// ─────────────────────────────────────────────────────────────────────────────
// Tracking event
// ─────────────────────────────────────────────────────────────────────────────

void MapperNode::onTrackingEvent(orb_slam3_msgs::msg::TrackingEvent::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->type == orb_slam3_msgs::msg::TrackingEvent::LOST) {
    was_lost_ = true;
  } else if (msg->type == orb_slam3_msgs::msg::TrackingEvent::RELOCALIZED ||
             msg->type == orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED) {
    was_lost_ = false;
    deleteProvisionalMarkers();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Scan
// ─────────────────────────────────────────────────────────────────────────────

void MapperNode::onScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  ++scans_received_;
  const int64_t stamp_ns = toNs(msg->header.stamp);
  const auto end_ns = scanEndStampNs(*msg);
  if (!end_ns) {
    publishDiagnostics("WARN", "scan rejected: invalid per-ray timing");
    return;
  }
  const std::string scan_frame = msg->header.frame_id;

  // Resolve the mount at acquisition time before queuing. Processing itself is
  // deferred until wheel odometry covers the final ray and a future visual
  // frame brackets the complete sweep.
  Pose2 base_to_lidar;
  try {
    const rclcpp::Time scan_stamp(msg->header.stamp);
    const auto tf = tf_buffer_->lookupTransform(
        base_frame_, scan_frame, scan_stamp);

    const double roll = std::atan2(
        2.0 * (tf.transform.rotation.w * tf.transform.rotation.x +
               tf.transform.rotation.y * tf.transform.rotation.z),
        1.0 - 2.0 * (tf.transform.rotation.x * tf.transform.rotation.x +
                     tf.transform.rotation.y * tf.transform.rotation.y));
    const double pitch = std::asin(std::clamp(
        2.0 * (tf.transform.rotation.w * tf.transform.rotation.y -
               tf.transform.rotation.z * tf.transform.rotation.x),
        -1.0, 1.0));
    const double max_rp = max_roll_pitch_deg_ * M_PI / 180.0;
    const double height = std::abs(tf.transform.translation.z);
    if (std::abs(roll) > max_rp || std::abs(pitch) > max_rp || height > max_height_delta_m_) {
      ++planarity_rejections_;
      publishDiagnostics("WARN", "scan rejected: non-planar TF base_link->" + scan_frame);
      return;
    }
    base_to_lidar = poseFromTransform(tf.transform);
  } catch (const tf2::TransformException& ex) {
    ++tf_lookup_failures_;
    publishDiagnostics("ERROR",
        std::string("TF lookup failed base_link->") + scan_frame + ": " + ex.what());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_scans_.size() >= static_cast<std::size_t>(pending_scan_limit_)) {
    pending_scans_.pop_front();
    ++scans_timeout_dropped_;
    publishDiagnostics("WARN", "pending scan queue full: dropped oldest scan");
  }
  pending_scans_.push_back({msg, base_to_lidar, stamp_ns, *end_ns, was_lost_});
  processPendingScansLocked();
}

void MapperNode::processPendingScansLocked() {
  for (auto scan = pending_scans_.begin(); scan != pending_scans_.end();) {
    const PendingScanResult result = processPendingScanLocked(*scan);
    if (result == PendingScanResult::kWaiting) {
      ++scan;
    } else {
      scan = pending_scans_.erase(scan);
    }
  }
}

MapperNode::PendingScanResult MapperNode::processPendingScanLocked(
  const PendingScan& pending) {
  const int64_t timeout_ns = static_cast<int64_t>(pending_scan_timeout_s_ * 1e9);
  const int64_t anchor_gap_ns = static_cast<int64_t>(visual_anchor_max_gap_ms_ * 1e6);
  const auto newest_wheel = wheel_buf_->newestStamp();

  // maximumYawExcursion() also verifies interpolation coverage across the full
  // scan interval. No directional prefix is ever archived.
  const auto yaw_excursion = wheel_buf_->maximumYawExcursion(
    pending.start_ns, pending.end_ns);
  if (!yaw_excursion) {
    if (advancedBeyond(newest_wheel, pending.end_ns, timeout_ns)) {
      ++scans_no_wheel_;
      ++scans_timeout_dropped_;
      publishDiagnostics("WARN", "scan dropped: complete wheel coverage timed out");
      return PendingScanResult::kDropped;
    }
    return PendingScanResult::kWaiting;
  }
  if (*yaw_excursion > max_scan_yaw_change_rad_) {
    ++scans_turn_rejected_;
    publishDiagnostics("WARN", "scan rejected: yaw excursion exceeds configured limit");
    return PendingScanResult::kDropped;
  }

  ScanValue scan;
  scan.id = static_cast<uint64_t>(pending.start_ns);
  scan.stamp_ns = pending.start_ns;
  scan.angle_min = pending.message->angle_min;
  scan.angle_increment = pending.message->angle_increment;
  scan.time_increment = pending.message->time_increment;
  scan.range_min = pending.message->range_min;
  scan.range_max = pending.message->range_max;
  scan.ranges = pending.message->ranges;

  const bool loss_scan = pending.tracking_lost ||
    traj_->isLossTimestamp(pending.start_ns);
  std::optional<ScanPose> placement;
  std::vector<Ray2> local_rays;
  std::optional<ArchivedBracketedMotion> archived_motion;

  if (loss_scan) {
    // LOST intervals keep the established wheel-only provisional geometry;
    // timestamp-based membership survives callback reordering and recovery.
    auto rays = ScanDeskewer::deskew(
      scan, Pose2{}, pending.base_to_lidar, *wheel_buf_);
    if (!rays || rays->empty()) {
      return PendingScanResult::kDropped;
    }
    local_rays = std::move(*rays);
    placement = traj_->placeScan(
      pending.start_ns, pending.tracking_lost &&
        !traj_->isLossTimestamp(pending.start_ns));
  } else {
    const auto bracket = traj_->visualWheelBracket(
      pending.start_ns, pending.end_ns, anchor_gap_ns);
    if (bracket) {
      const ScanMotionBracket motion_bracket{
        bracket->start_stamp_ns, bracket->end_stamp_ns,
        bracket->start_map_pose, bracket->end_map_pose,
        bracket->start_wheel_pose, bracket->end_wheel_pose};
      auto deskewed = ScanDeskewer::deskewBracketed(
        scan, pending.base_to_lidar, *wheel_buf_, motion_bracket);
      if (!deskewed || deskewed->rays.empty()) {
        return PendingScanResult::kDropped;
      }
      local_rays = std::move(deskewed->rays);
      archived_motion = ArchivedBracketedMotion{
        bracket->start_frame_index, bracket->end_frame_index,
        pending.base_to_lidar, std::move(deskewed->ray_motions)};
      placement = traj_->placeBracketedScan(
        pending.start_ns, pending.end_ns, anchor_gap_ns);
    }
  }

  if (!placement) {
    // Once wheel coverage is complete, wheel progress cannot make a valid
    // delayed visual bracket impossible. Expire only when visual sensor time
    // itself has advanced beyond the bounded waiting window.
    const bool expired = advancedBeyond(
      traj_->latestFrameStamp(), pending.end_ns, timeout_ns);
    if (expired) {
      ++scans_no_anchor_;
      ++scans_timeout_dropped_;
      publishDiagnostics("WARN", "scan dropped: visual anchor bracket timed out");
      return PendingScanResult::kDropped;
    }
    return PendingScanResult::kWaiting;
  }

  ArchivedScan archived;
  archived.scan_id = placement->scan_id;
  archived.stamp_ns = pending.start_ns;
  archived.rays = local_rays;
  archived.bracketed_motion = std::move(archived_motion);
  archive_->scans.push_back(archived);

  if (placement->committed) {
    if (committed_scan_ids_.find(placement->scan_id) == committed_scan_ids_.end()) {
      rebuilder_->appendCommitted(archived, *placement, last_graph_revision_);
      committed_scan_ids_.insert(placement->scan_id);
    }
    ++scans_committed_;
    publishCommittedScanMarker(local_rays, placement->pose, pending.start_ns);
  } else {
    ++scans_provisional_;
    publishProvisionalScanMarker(local_rays, placement->pose, pending.start_ns);
  }
  return PendingScanResult::kProcessed;
}

// ─────────────────────────────────────────────────────────────────────────────
// Map / revision publish (called from MapRebuilder worker thread — no mutex)
// ─────────────────────────────────────────────────────────────────────────────

void MapperNode::publishMapAndRevision(std::shared_ptr<const MapSnapshot> snapshot,
                                        const RebuildStatus& status) {
  if (!snapshot) return;

  mapper_node_test_hooks::beforePublish();

  const auto publish_stamp = now();
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = map_frame_;
  grid.header.stamp    = publish_stamp;
  grid.info.resolution = static_cast<float>(snapshot->grid.resolution_m);
  grid.info.width      = snapshot->grid.width;
  grid.info.height     = snapshot->grid.height;
  grid.info.origin.position.x = snapshot->grid.origin_x;
  grid.info.origin.position.y = snapshot->grid.origin_y;
  grid.info.origin.orientation.w = 1.0;
  grid.data = snapshot->grid.cells;
  map_pub_->publish(grid);

  orb_slam3_msgs::msg::MapRevision rev;
  rev.header.frame_id = map_frame_;
  rev.header.stamp    = publish_stamp;
  switch (status.state) {
    case RebuildState::kIdle:      rev.state = orb_slam3_msgs::msg::MapRevision::IDLE;      break;
    case RebuildState::kBuilding:  rev.state = orb_slam3_msgs::msg::MapRevision::BUILDING;  break;
    case RebuildState::kPublished: rev.state = orb_slam3_msgs::msg::MapRevision::PUBLISHED; break;
    case RebuildState::kFailed:    rev.state = orb_slam3_msgs::msg::MapRevision::FAILED;    break;
  }
  rev.graph_revision      = status.graph_revision;
  rev.map_revision        = status.map_revision;
  rev.input_scan_count    = status.input_scan_count;
  rev.committed_scan_count = status.committed_scan_count;
  rev.duration_ms         = status.duration_ms;
  rev.detail              = status.detail;
  map_rev_pub_->publish(rev);
  mapper_node_test_hooks::afterPublish();
}

// ─────────────────────────────────────────────────────────────────────────────
// Corrected path
// ─────────────────────────────────────────────────────────────────────────────

void MapperNode::publishCorrectedPath(std::shared_ptr<const TrajectoryRevision> revision) {
  if (!revision) return;
  std_msgs::msg::Header hdr;
  hdr.frame_id = map_frame_;
  hdr.stamp    = now();

  nav_msgs::msg::Path path;
  path.header = hdr;
  orb_slam3_msgs::msg::RevisionedPath rev_path;
  rev_path.header        = hdr;
  rev_path.graph_revision = revision->graph_revision;

  for (const auto& sp : revision->scans) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = map_frame_;
    // Per-pose stamp from the scan's own timestamp so downstream consumers can
    // match poses across runs by time; the message-level header carries publish
    // time and frame. sp.stamp_ns is nanoseconds since epoch.
    ps.header.stamp.sec     = static_cast<int32_t>(sp.stamp_ns / 1'000'000'000LL);
    ps.header.stamp.nanosec = static_cast<uint32_t>(sp.stamp_ns % 1'000'000'000LL);
    ps.pose.position.x  = sp.pose.x;
    ps.pose.position.y  = sp.pose.y;
    ps.pose.orientation.w = std::cos(sp.pose.yaw * 0.5);
    ps.pose.orientation.z = std::sin(sp.pose.yaw * 0.5);
    path.poses.push_back(ps);
    rev_path.poses.push_back(ps);
  }

  corrected_path_pub_->publish(path);
  corrected_path_rev_pub_->publish(rev_path);
}

// ─────────────────────────────────────────────────────────────────────────────
// Markers
// ─────────────────────────────────────────────────────────────────────────────

void MapperNode::publishCommittedScanMarker(const std::vector<Ray2>& rays,
                                             const Pose2& pose,
                                             int64_t stamp_ns) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id    = map_frame_;
  marker.header.stamp.sec   = static_cast<int32_t>(stamp_ns / 1'000'000'000LL);
  marker.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1'000'000'000LL);
  marker.ns     = "committed_scan";
  marker.id     = 0;
  marker.type   = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.05; marker.scale.y = 0.05;
  marker.color.r = 0.0f; marker.color.g = 1.0f;
  marker.color.b = 0.0f; marker.color.a = 1.0f;
  for (const auto& ray : rays) {
    if (ray.has_hit) {
      const Point2 end = transformLocal(pose, ray.end);
      geometry_msgs::msg::Point p;
      p.x = end.x; p.y = end.y; p.z = 0.0;
      marker.points.push_back(p);
    }
  }
  committed_scan_pub_->publish(marker);
}

void MapperNode::publishProvisionalScanMarker(const std::vector<Ray2>& rays,
                                               const Pose2& pose,
                                               int64_t stamp_ns) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id    = map_frame_;
  marker.header.stamp.sec   = static_cast<int32_t>(stamp_ns / 1'000'000'000LL);
  marker.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1'000'000'000LL);
  marker.ns     = "provisional_scan";
  marker.id     = 0;
  marker.type   = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.05; marker.scale.y = 0.05;
  marker.color.r = 1.0f; marker.color.g = 1.0f;  // yellow
  marker.color.b = 0.0f; marker.color.a = 1.0f;
  for (const auto& ray : rays) {
    if (ray.has_hit) {
      const Point2 end = transformLocal(pose, ray.end);
      geometry_msgs::msg::Point p;
      p.x = end.x; p.y = end.y; p.z = 0.0;
      marker.points.push_back(p);
    }
  }
  provisional_scan_pub_->publish(marker);
}

void MapperNode::deleteProvisionalMarkers() {
  visualization_msgs::msg::Marker del;
  del.header.frame_id = map_frame_;
  del.header.stamp    = now();
  del.ns     = "provisional_scan";
  del.id     = 0;
  del.action = visualization_msgs::msg::Marker::DELETE;
  provisional_scan_pub_->publish(del);
}

// ─────────────────────────────────────────────────────────────────────────────
// Diagnostics
// ─────────────────────────────────────────────────────────────────────────────

void MapperNode::publishDiagnostics(const std::string& level, const std::string& message) {
  diagnostic_msgs::msg::DiagnosticArray array;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name    = get_name();
  status.message = message;
  status.level   = (level == "ERROR") ? 2 : (level == "WARN") ? 1 : 0;

  auto add_kv = [&status](const std::string& key, uint64_t value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = std::to_string(value);
    status.values.push_back(kv);
  };
  add_kv("tf_lookup_failures", tf_lookup_failures_.load());
  add_kv("wheel_interp_failures", wheel_interp_failures_.load());
  add_kv("planarity_rejections", planarity_rejections_.load());
  add_kv("scans_received", scans_received_.load());
  add_kv("scans_no_wheel", scans_no_wheel_.load());
  add_kv("scans_no_anchor", scans_no_anchor_.load());
  add_kv("scans_committed", scans_committed_.load());
  add_kv("scans_provisional", scans_provisional_.load());
  add_kv("scans_turn_rejected", scans_turn_rejected_.load());
  add_kv("scans_timeout_dropped", scans_timeout_dropped_.load());

  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

}  // namespace orb_lidar_mapper
