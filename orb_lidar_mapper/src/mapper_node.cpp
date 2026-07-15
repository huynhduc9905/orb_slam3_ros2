#include "orb_lidar_mapper/mapper_node.hpp"

#include <algorithm>
#include <cmath>
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
  usable_range_m_(declare_parameter("usable_range_m", 12.0)),
  max_roll_pitch_deg_(declare_parameter("max_roll_pitch_deg", 10.0)),
  max_height_delta_m_(declare_parameter("max_height_delta_m", 0.15)) {

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
  auto sensor_qos  = rclcpp::SensorDataQoS();
  auto reliable_tl = rclcpp::QoS(1).reliable().transient_local();
  auto reliable    = rclcpp::QoS(100).reliable();
  auto reliable10  = rclcpp::QoS(10).reliable();

  // Publishers
  map_pub_  = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, reliable_tl);
  map_rev_pub_ = create_publisher<orb_slam3_msgs::msg::MapRevision>(
      "/orb_lidar/map_revision", reliable_tl);
  corrected_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/orb_lidar/corrected_path", reliable_tl);
  corrected_path_rev_pub_ = create_publisher<orb_slam3_msgs::msg::RevisionedPath>(
      "/orb_lidar/corrected_path_revisioned", reliable_tl);
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
      graph_snapshot_topic_, reliable_tl,
      [this](orb_slam3_msgs::msg::GraphSnapshot::ConstSharedPtr msg) { onGraphSnapshot(msg); });

  event_sub_ = create_subscription<orb_slam3_msgs::msg::TrackingEvent>(
      tracking_event_topic_, reliable,
      [this](orb_slam3_msgs::msg::TrackingEvent::ConstSharedPtr msg) { onTrackingEvent(msg); });
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
  if (msg->pose_valid) {
    anchor.map_pose = poseFromMsg(msg->pose);
    anchor.reference_to_frame = poseFromTransform(msg->reference_to_frame);
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Populate wheel_pose by interpolating the mirror buffer at the frame timestamp.
  // Identity default would poison the anchor (map_pose * I^{-1} * scan_wheel).
  const auto wheel_at_frame = wheel_buf_->interpolate(stamp_ns);
  if (!wheel_at_frame) {
    ++wheel_interp_failures_;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "wheel interpolation failed at tracked-frame stamp %ld ns; skipping anchor",
        static_cast<long>(stamp_ns));
    publishDiagnostics("WARN", "tracked frame skipped: wheel interpolation failed");
    return;
  }
  anchor.wheel_pose = *wheel_at_frame;
  traj_->addTrackedFrame(anchor);
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
  const std::string scan_frame = msg->header.frame_id;

  // TF lookup at the scan stamp (not latest) so dynamic mounts stay consistent.
  // Static TFs remain valid at any stamp, so tests with a static broadcaster still resolve.
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

  ScanValue sv;
  sv.id             = static_cast<uint64_t>(stamp_ns);
  sv.stamp_ns       = stamp_ns;
  sv.angle_min      = msg->angle_min;
  sv.angle_increment = msg->angle_increment;
  sv.time_increment = msg->time_increment;
  sv.range_min      = msg->range_min;
  sv.range_max      = msg->range_max;
  sv.ranges         = msg->ranges;

  std::lock_guard<std::mutex> lock(mutex_);

  // Distinguish the two placeScan failure modes for diagnostics.
  const bool have_wheel = wheel_buf_->interpolate(stamp_ns).has_value();

  // Place scan in trajectory store
  const auto placement = traj_->placeScan(stamp_ns);
  if (!placement.has_value()) {
    if (!have_wheel) {
      ++scans_no_wheel_;
    } else {
      ++scans_no_anchor_;
    }
    return;
  }
  const ScanPose scan_pose = *placement;

  // Deskew into the scan-start base frame (identity committed pose).
  // MapRebuilder applies the corrected ScanPose later when inserting rays.
  auto rays_opt = ScanDeskewer::deskew(sv, Pose2{}, base_to_lidar, *wheel_buf_);
  if (!rays_opt.has_value() || rays_opt->empty()) {
    return;
  }
  const std::vector<Ray2>& local_rays = *rays_opt;

  // Archive rays in the local base frame.
  ArchivedScan archived;
  archived.scan_id  = scan_pose.scan_id;
  archived.stamp_ns = stamp_ns;
  archived.rays     = local_rays;
  archive_->scans.push_back(archived);

  if (scan_pose.committed && !was_lost_) {
    if (committed_scan_ids_.find(scan_pose.scan_id) == committed_scan_ids_.end()) {
      rebuilder_->appendCommitted(archived, scan_pose, last_graph_revision_);
      committed_scan_ids_.insert(scan_pose.scan_id);
    }
    ++scans_committed_;
    publishCommittedScanMarker(local_rays, scan_pose.pose, stamp_ns);
  } else {
    // Provisional (lost or not yet committed).
    ++scans_provisional_;
    publishProvisionalScanMarker(local_rays, scan_pose.pose, stamp_ns);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Map / revision publish (called from MapRebuilder worker thread — no mutex)
// ─────────────────────────────────────────────────────────────────────────────

void MapperNode::publishMapAndRevision(std::shared_ptr<const MapSnapshot> snapshot,
                                        const RebuildStatus& status) {
  if (!snapshot) return;

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = map_frame_;
  grid.header.stamp    = now();
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
  rev.header.stamp    = now();
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

  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

}  // namespace orb_lidar_mapper
