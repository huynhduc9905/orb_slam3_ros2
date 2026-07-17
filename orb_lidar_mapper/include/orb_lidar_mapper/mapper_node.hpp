#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <orb_slam3_msgs/msg/graph_snapshot.hpp>
#include <orb_slam3_msgs/msg/map_revision.hpp>
#include <orb_slam3_msgs/msg/revisioned_path.hpp>
#include <orb_slam3_msgs/msg/tracked_frame.hpp>
#include <orb_slam3_msgs/msg/tracking_event.hpp>

#include "orb_lidar_mapper/map_rebuilder.hpp"
#include "orb_lidar_mapper/scan_deskewer.hpp"
#include "orb_lidar_mapper/trajectory_store.hpp"

namespace orb_lidar_mapper {

class MapperNode final : public rclcpp::Node {
public:
  explicit MapperNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~MapperNode() override;

private:
  struct PendingScan {
    sensor_msgs::msg::LaserScan::ConstSharedPtr message;
    Pose2 base_to_lidar;
    std::int64_t start_ns{};
    std::int64_t end_ns{};
    bool tracking_lost{};
  };

  enum class PendingScanResult { kWaiting, kProcessed, kDropped };

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onTrackedFrame(orb_slam3_msgs::msg::TrackedFrame::ConstSharedPtr msg);
  void onGraphSnapshot(orb_slam3_msgs::msg::GraphSnapshot::ConstSharedPtr msg);
  void onTrackingEvent(orb_slam3_msgs::msg::TrackingEvent::ConstSharedPtr msg);
  void onScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  // ── Internal helpers ───────────────────────────────────────────────────────
  void processPendingScansLocked();
  PendingScanResult processPendingScanLocked(const PendingScan& pending);
  void publishCommittedScanMarker(const std::vector<Ray2>& rays, const Pose2& pose, int64_t stamp_ns);
  void publishProvisionalScanMarker(const std::vector<Ray2>& rays, const Pose2& pose, int64_t stamp_ns);
  void deleteProvisionalMarkers();
  void publishMapAndRevision(std::shared_ptr<const MapSnapshot> snapshot,
                              const RebuildStatus& status);
  void publishCorrectedPath(std::shared_ptr<const TrajectoryRevision> revision);
  void publishDiagnostics(const std::string& level, const std::string& message);

  // ── Parameters ────────────────────────────────────────────────────────────
  std::string odom_topic_;
  std::string scan_topic_;
  std::string tracked_frame_topic_;
  std::string graph_snapshot_topic_;
  std::string tracking_event_topic_;
  std::string map_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double wheel_retention_s_;
  double wheel_max_gap_ms_;
  double resolution_m_;
  double usable_range_m_;
  double hit_range_max_m_;
  double hit_log_odds_;
  double miss_log_odds_;
  double max_roll_pitch_deg_;
  double max_height_delta_m_;
  double max_scan_yaw_change_rad_;
  double visual_anchor_max_gap_ms_;
  double pending_scan_timeout_s_;
  std::int64_t pending_scan_limit_;

  // ── Core objects (all accessed only from subscription callbacks / mutex) ──
  std::mutex mutex_;
  std::unique_ptr<TrajectoryStore> traj_;
  std::unique_ptr<TimedPoseBuffer> wheel_buf_;   // mirror for deskewing
  std::unique_ptr<MapRebuilder> rebuilder_;
  std::shared_ptr<ScanArchive> archive_;
  std::deque<PendingScan> pending_scans_;
  std::unordered_set<uint64_t> committed_scan_ids_;  // scan_ids already fed to rebuilder

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Tracking state
  bool was_lost_{false};       // true while we are in a LOST interval
  uint64_t last_graph_revision_{0};

  // Diagnostic counters
  std::atomic<uint64_t> tf_lookup_failures_{0};
  std::atomic<uint64_t> wheel_interp_failures_{0};
  std::atomic<uint64_t> planarity_rejections_{0};
  // Scan-pipeline instrumentation (diagnosing low commit rate)
  std::atomic<uint64_t> scans_received_{0};
  std::atomic<uint64_t> scans_no_wheel_{0};
  std::atomic<uint64_t> scans_no_anchor_{0};
  std::atomic<uint64_t> scans_committed_{0};
  std::atomic<uint64_t> scans_provisional_{0};
  std::atomic<uint64_t> scans_turn_rejected_{0};
  std::atomic<uint64_t> scans_timeout_dropped_{0};

  // Wheel path accumulation (guarded by mutex_)
  std::vector<geometry_msgs::msg::PoseStamped> wheel_poses_;

  // ── Subscriptions ─────────────────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<orb_slam3_msgs::msg::TrackedFrame>::SharedPtr tracked_sub_;
  rclcpp::Subscription<orb_slam3_msgs::msg::GraphSnapshot>::SharedPtr graph_sub_;
  rclcpp::Subscription<orb_slam3_msgs::msg::TrackingEvent>::SharedPtr event_sub_;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<orb_slam3_msgs::msg::MapRevision>::SharedPtr map_rev_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_path_pub_;
  rclcpp::Publisher<orb_slam3_msgs::msg::RevisionedPath>::SharedPtr corrected_path_rev_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr wheel_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr provisional_scan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr committed_scan_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace orb_lidar_mapper
