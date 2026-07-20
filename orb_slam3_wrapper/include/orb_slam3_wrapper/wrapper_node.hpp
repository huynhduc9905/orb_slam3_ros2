#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>

#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

#include <orb_slam3_msgs/msg/graph_snapshot.hpp>
#include <orb_slam3_msgs/msg/keyframe_pose.hpp>
#include <orb_slam3_msgs/msg/loop_edge.hpp>
#include <orb_slam3_msgs/msg/tracked_frame.hpp>
#include <orb_slam3_msgs/msg/tracking_event.hpp>

#include <opencv2/core/utility.hpp>

#include "orb_slam3_wrapper/backend.hpp"
#include "orb_slam3_wrapper/calibration.hpp"
#include "orb_slam3_wrapper/pose_conversion.hpp"
#include "orb_slam3_wrapper/latest_image_worker.hpp"

namespace orb_slam3_wrapper {

std::size_t canonicalLoopEdgeCountForTest(const ORB_SLAM3::GraphSnapshot& graph);

class WrapperNode final : public rclcpp::Node {
public:
  explicit WrapperNode(std::unique_ptr<SlamBackend> backend = nullptr);
  ~WrapperNode() override;

  void processStereoForTest(const sensor_msgs::msg::Image& left,
                            const sensor_msgs::msg::Image& right);
  const orb_slam3_msgs::msg::TrackedFrame& lastTrackedFrameForTest() const { return last_tracked_; }
  const orb_slam3_msgs::msg::GraphSnapshot& lastGraphSnapshotForTest() const { return last_graph_; }
  const orb_slam3_msgs::msg::TrackingEvent& lastTrackingEventForTest() const { return last_event_; }
  void setCameraInfoForTest(const sensor_msgs::msg::CameraInfo& left,
                            const sensor_msgs::msg::CameraInfo& right);
  std::size_t graphPublishCountForTest() const { return graph_publish_count_; }
  std::size_t eventPublishCountForTest() const { return event_publish_count_; }

private:
  using Image = sensor_msgs::msg::Image;
  void imageCallback(const Image::ConstSharedPtr left, const Image::ConstSharedPtr right);
  void infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, bool left);
  void processStereo(const Image& left, const Image& right);
  void pollGraphChanges();
  cv::Mat mono8(const Image& image) const;
  void publishGraph(const ORB_SLAM3::GraphSnapshot& graph, const std_msgs::msg::Header& header);
  void publishDiagnostics(const std::string& level, const std::string& message);
  bool resolveExtrinsic(const std::string& image_frame, const rclcpp::Time& stamp);

  std::unique_ptr<SlamBackend> backend_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string base_frame_;
  std::string map_frame_;
  double sync_max_skew_ms_;
  double tracking_image_rate_hz_;
  int opencv_num_threads_;
  std::string left_info_topic_;
  std::string right_info_topic_;
  std::optional<sensor_msgs::msg::CameraInfo> left_info_;
  std::optional<sensor_msgs::msg::CameraInfo> right_info_;
  std::optional<StereoCalibration> calibration_;
  std::optional<PoseConverter> converter_;
  bool backend_configured_{false};
  std::optional<StereoCalibration> failed_calibration_;
  std::string last_configuration_error_;
  int last_tracking_state_{-1};
  std::uint64_t last_graph_revision_{0};
  rclcpp::Time last_tracking_image_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<orb_slam3_msgs::msg::TrackedFrame>::SharedPtr tracked_pub_;
  rclcpp::Publisher<orb_slam3_msgs::msg::GraphSnapshot>::SharedPtr graph_pub_;
  rclcpp::Publisher<orb_slam3_msgs::msg::TrackingEvent>::SharedPtr events_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loops_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  std::unique_ptr<message_filters::Subscriber<Image>> left_sync_sub_;
  std::unique_ptr<message_filters::Subscriber<Image>> right_sync_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  orb_slam3_msgs::msg::TrackedFrame last_tracked_;
  orb_slam3_msgs::msg::GraphSnapshot last_graph_;
  orb_slam3_msgs::msg::TrackingEvent last_event_;
  std::size_t graph_publish_count_{0};
  std::size_t event_publish_count_{0};
  bool graph_baseline_captured_{false};
  std::optional<ORB_SLAM3::GraphSnapshot> previous_graph_;
  std::unique_ptr<LatestImageWorker> image_worker_;
};

}  // namespace orb_slam3_wrapper
