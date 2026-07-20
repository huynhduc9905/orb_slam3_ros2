#include "orb_slam3_wrapper/wrapper_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include <tf2/exceptions.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unordered_map>
#include <set>

#include "orb_slam3_wrapper/graph_semantics.hpp"

namespace orb_slam3_wrapper {
namespace {

geometry_msgs::msg::Pose poseMsg(const Eigen::Isometry3d& pose) {
  geometry_msgs::msg::Pose msg;
  const auto q = Eigen::Quaterniond(pose.rotation());
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  msg.orientation.x = q.x(); msg.orientation.y = q.y();
  msg.orientation.z = q.z(); msg.orientation.w = q.w();
  return msg;
}

geometry_msgs::msg::Transform transformMsg(const Eigen::Isometry3d& pose) {
  geometry_msgs::msg::Transform msg;
  const auto q = Eigen::Quaterniond(pose.rotation());
  msg.translation.x = pose.translation().x();
  msg.translation.y = pose.translation().y();
  msg.translation.z = pose.translation().z();
  msg.rotation.x = q.x(); msg.rotation.y = q.y();
  msg.rotation.z = q.z(); msg.rotation.w = q.w();
  return msg;
}

Eigen::Isometry3d tfToEigen(const geometry_msgs::msg::Transform& tf) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() << tf.translation.x, tf.translation.y, tf.translation.z;
  result.linear() = Eigen::Quaterniond(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z).toRotationMatrix();
  return result;
}

bool cameraInfoCalibrationEqual(const sensor_msgs::msg::CameraInfo& lhs,
                                const sensor_msgs::msg::CameraInfo& rhs) {
  return lhs.width == rhs.width && lhs.height == rhs.height &&
      lhs.distortion_model == rhs.distortion_model && lhs.d == rhs.d &&
      lhs.k == rhs.k && lhs.r == rhs.r && lhs.p == rhs.p &&
      lhs.binning_x == rhs.binning_x && lhs.binning_y == rhs.binning_y &&
      lhs.roi.x_offset == rhs.roi.x_offset && lhs.roi.y_offset == rhs.roi.y_offset &&
      lhs.roi.height == rhs.roi.height && lhs.roi.width == rhs.roi.width &&
      lhs.roi.do_rectify == rhs.roi.do_rectify;
}

std::size_t canonicalLoopEdgeCount(const ORB_SLAM3::GraphSnapshot& graph) {
  std::set<std::pair<std::uint64_t, std::uint64_t>> edges;
  for (const auto& keyframe : graph.keyframes) {
    for (const auto id : keyframe.loop_edge_ids) {
      edges.emplace(std::min(keyframe.id, id), std::max(keyframe.id, id));
    }
  }
  return edges.size();
}

}  // namespace

std::size_t canonicalLoopEdgeCountForTest(const ORB_SLAM3::GraphSnapshot& graph) {
  return canonicalLoopEdgeCount(graph);
}

WrapperNode::WrapperNode(std::unique_ptr<SlamBackend> backend)
: Node("orb_slam3_wrapper"), backend_(std::move(backend)),
  tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
  tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_)),
  base_frame_(declare_parameter("base_frame", "base_link")),
  map_frame_(declare_parameter("map_frame", "orb_map")),
  sync_max_skew_ms_(declare_parameter("sync_max_skew_ms", 5.0)),
  tracking_image_rate_hz_(declare_parameter("tracking_image_rate_hz", 5.0)),
  opencv_num_threads_(declare_parameter("opencv_num_threads", 4)),
  left_info_topic_(declare_parameter("left_info_topic", "/camera/camera/infra1/camera_info")),
  right_info_topic_(declare_parameter("right_info_topic", "/camera/camera/infra2/camera_info")) {
  // OpenCV/TBB defaults to hardware_concurrency() worker threads for every
  // parallel_for_ dispatch (stereo rectification, per-octave ORB feature
  // extraction) — on a shared machine that means a fresh 16-wide thread-pool
  // fan-out on every single stereo frame, competing with every other node's
  // callbacks for the same cores and adding thread wake/join overhead that
  // can dominate the actual per-frame work. Cap it to a small, configurable
  // pool so this node leaves headroom for the rest of the stack.
  cv::setNumThreads(opencv_num_threads_);
  const auto left_topic = declare_parameter("left_image_topic", "/camera/camera/infra1/image_rect_raw");
  const auto right_topic = declare_parameter("right_image_topic", "/camera/camera/infra2/image_rect_raw");
  std::string settings_default;
  std::string vocabulary_default;
  try {
    settings_default = ament_index_cpp::get_package_share_directory("orb_slam3_wrapper") + "/config/tasterobot_stereo.yaml";
    vocabulary_default = ament_index_cpp::get_package_share_directory("orb_slam3_vendor") + "/vocabulary/ORBvoc.txt";
  } catch (const std::exception&) {
    settings_default = ""; vocabulary_default = "";
  }
  const auto settings = declare_parameter("settings_file", settings_default);
  const auto vocabulary = declare_parameter("vocabulary_file", vocabulary_default);
  if (!has_parameter("use_sim_time")) declare_parameter("use_sim_time", false);

  auto sensor_qos = rclcpp::SensorDataQoS();
  tracked_pub_ = create_publisher<orb_slam3_msgs::msg::TrackedFrame>("/orb_slam3/tracked_frame", 10);
  graph_pub_ = create_publisher<orb_slam3_msgs::msg::GraphSnapshot>(
      "/orb_slam3/graph_snapshot", rclcpp::QoS(1).reliable().transient_local());
  events_pub_ = create_publisher<orb_slam3_msgs::msg::TrackingEvent>(
      "/orb_slam3/events", rclcpp::QoS(100).reliable());
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/orb_slam3/path", 10);
  keyframes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/orb_slam3/keyframes", 10);
  loops_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/orb_slam3/loop_edges", 10);
  image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/orb_slam3/tracking_image/compressed", 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  image_worker_ = std::make_unique<LatestImageWorker>([this](const std::vector<unsigned char>& encoded,
                                                             const std_msgs::msg::Header& header) {
    if (!rclcpp::ok()) return;
    sensor_msgs::msg::CompressedImage image;
    image.header = header;
    image.format = "jpeg";
    image.data = encoded;
    image_pub_->publish(image);
  }, LatestImageWorker::Encoder{}, [this](const std::string& error) {
    if (rclcpp::ok()) publishDiagnostics("ERROR", std::string("tracking image worker: ") + error);
  });

  left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(left_info_topic_, sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { infoCallback(msg, true); });
  right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(right_info_topic_, sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { infoCallback(msg, false); });
  left_sync_sub_ = std::make_unique<message_filters::Subscriber<Image>>(
      this, left_topic, sensor_qos.get_rmw_qos_profile());
  right_sync_sub_ = std::make_unique<message_filters::Subscriber<Image>>(
      this, right_topic, sensor_qos.get_rmw_qos_profile());
  synchronizer_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), *left_sync_sub_, *right_sync_sub_);
  synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_max_skew_ms_ / 1000.0));
  synchronizer_->registerCallback([this](Image::ConstSharedPtr left, Image::ConstSharedPtr right) {
    imageCallback(left, right);
  });

  graph_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
    pollGraphChanges();
  });

  if (!backend_ && !settings.empty() && !vocabulary.empty()) {
    backend_ = std::make_unique<OrbSlam3Backend>(vocabulary, settings);
  }
}

WrapperNode::~WrapperNode() { if (image_worker_) image_worker_->stop(); }

void WrapperNode::infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, bool left) {
  auto& cached = left ? left_info_ : right_info_;
  if (cached && cameraInfoCalibrationEqual(*cached, *msg)) return;
  cached = *msg;
  calibration_.reset();
  backend_configured_ = false;
  graph_baseline_captured_ = false;
  previous_graph_.reset();
  last_graph_revision_ = 0;
  failed_calibration_.reset();
}

void WrapperNode::setCameraInfoForTest(const sensor_msgs::msg::CameraInfo& left,
                                       const sensor_msgs::msg::CameraInfo& right) {
  const bool changed = !left_info_ || !right_info_ ||
      !cameraInfoCalibrationEqual(*left_info_, left) ||
      !cameraInfoCalibrationEqual(*right_info_, right);
  left_info_ = left;
  right_info_ = right;
  if (!changed) return;
  calibration_.reset();
  backend_configured_ = false;
  graph_baseline_captured_ = false;
  previous_graph_.reset();
  last_graph_revision_ = 0;
  failed_calibration_.reset();
}

void WrapperNode::imageCallback(const Image::ConstSharedPtr left, const Image::ConstSharedPtr right) {
  // ApproximateTime performs the bounded queue-10 pairing and max-skew rejection.
  if (left && right) processStereo(*left, *right);
}

cv::Mat WrapperNode::mono8(const Image& image) const {
  if (image.encoding == "mono8" || image.encoding == "8UC1") {
    return cv::Mat(static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC1,
                   const_cast<unsigned char*>(image.data.data()), image.step);
  }
  return cv_bridge::toCvCopy(image, "mono8")->image;
}

bool WrapperNode::resolveExtrinsic(const std::string& image_frame, const rclcpp::Time& stamp) {
  try {
    const auto tf = tf_buffer_->lookupTransform(base_frame_, image_frame, stamp, rclcpp::Duration::from_seconds(0.1));
    converter_.emplace(tfToEigen(tf.transform));
    return true;
  } catch (const tf2::TransformException& error) {
    publishDiagnostics("ERROR", std::string("missing TF target=") + base_frame_ + " source=" + image_frame + ": " + error.what());
    return false;
  }
}

void WrapperNode::processStereoForTest(const Image& left, const Image& right) {
  if (!converter_) converter_.emplace(Eigen::Isometry3d::Identity());
  if (&left == &right) {
    auto right_copy = right;
    right_copy.header.frame_id = right_copy.header.frame_id.empty() ? "right_optical" : right_copy.header.frame_id + "_right";
    processStereo(left, right_copy);
  } else {
    processStereo(left, right);
  }
}

void WrapperNode::pollGraphChanges() {
  if (!backend_ || !backend_configured_ || last_tracked_.header.frame_id.empty()) return;
  if (!graph_baseline_captured_) {
    previous_graph_ = backend_->graphSnapshot();
    RCLCPP_INFO(get_logger(),
                "graph_observation stage=baseline revision=%lu raw_loop_edges=%zu previous_baseline=false",
                static_cast<unsigned long>(previous_graph_->revision),
                canonicalLoopEdgeCount(*previous_graph_));
    graph_baseline_captured_ = true;
  }
  if (!backend_->mapChanged()) return;
  const auto graph = backend_->graphSnapshot();
  if (graph.revision != last_graph_revision_) publishGraph(graph, last_tracked_.header);
}

void WrapperNode::processStereo(const Image& left, const Image& right) {
  if (!backend_) { publishDiagnostics("ERROR", "ORB backend is not configured"); return; }
  if (!left_info_ || !right_info_) { publishDiagnostics("ERROR", "stereo CameraInfo is not available"); return; }
  try {
    const auto candidate = Calibration::fromCameraInfo(*left_info_, *right_info_, left.header.frame_id, right.header.frame_id);
    if (!calibration_ || candidate.left_frame != calibration_->left_frame || candidate.right_frame != calibration_->right_frame ||
        candidate.width != calibration_->width || candidate.height != calibration_->height ||
        std::abs(candidate.baseline_m - calibration_->baseline_m) > 1e-9) {
      calibration_ = candidate;
      backend_configured_ = false;
      failed_calibration_.reset();
    }
  } catch (const std::exception& error) {
    const std::string message = error.what();
    if (message != last_configuration_error_) { publishDiagnostics("ERROR", message); last_configuration_error_ = message; }
    return;
  }
  if (!backend_configured_ && (!failed_calibration_ || failed_calibration_->left_frame != calibration_->left_frame ||
      failed_calibration_->right_frame != calibration_->right_frame || failed_calibration_->width != calibration_->width ||
      failed_calibration_->height != calibration_->height)) {
    std::string error;
    try {
      if (!backend_->configureCalibration(*calibration_, error)) {
        failed_calibration_ = calibration_;
        const auto message = error.empty() ? "backend calibration validation failed" : error;
        if (message != last_configuration_error_) { publishDiagnostics("ERROR", message); last_configuration_error_ = message; }
        return;
      }
    } catch (const std::exception& exception) {
      failed_calibration_ = calibration_;
      const std::string message = std::string("backend initialization failed: ") + exception.what();
      if (message != last_configuration_error_) { publishDiagnostics("ERROR", message); last_configuration_error_ = message; }
      return;
    } catch (...) {
      failed_calibration_ = calibration_;
      if (last_configuration_error_ != "backend initialization failed with an unknown error") {
        publishDiagnostics("ERROR", "backend initialization failed with an unknown error");
        last_configuration_error_ = "backend initialization failed with an unknown error";
      }
      return;
    }
    backend_configured_ = true;
    failed_calibration_.reset();
    last_configuration_error_.clear();
  }
  if (!backend_configured_) return;
  if (!converter_ && !resolveExtrinsic(left.header.frame_id, rclcpp::Time(left.header.stamp))) return;

  const auto stamp = rclcpp::Time(left.header.stamp);
  const auto frame = backend_->trackStereo(mono8(left), mono8(right), stamp.seconds());
  orb_slam3_msgs::msg::TrackedFrame output;
  output.header.stamp = left.header.stamp; output.header.frame_id = map_frame_;
  output.tracking_state = static_cast<std::uint8_t>(frame.tracking_state);
  output.pose_valid = frame.pose_valid;
  output.map_id = frame.map_id; output.reference_keyframe_id = frame.reference_keyframe_id;
  output.tracked_keypoints = static_cast<std::uint32_t>(frame.tracked_keypoints);
  output.graph_revision = last_graph_revision_;
  if (frame.pose_valid && converter_) {
    if (frame.tracking_state == orb_slam3_msgs::msg::TrackedFrame::OK && !converter_->initialized())
      converter_->anchor(frame.T_world_camera);
    if (converter_->initialized()) {
      output.pose = poseMsg(converter_->toBasePose(frame.T_world_camera));
      output.reference_to_frame = transformMsg(converter_->referenceToBaseFrame(
          frame.T_reference_camera_current_camera));
    } else if (frame.tracking_state == orb_slam3_msgs::msg::TrackedFrame::OK) {
      output.pose_valid = false;
    }
  }
  tracked_pub_->publish(output); last_tracked_ = output;

  if (last_tracking_state_ != -1 && last_tracking_state_ != frame.tracking_state) {
    orb_slam3_msgs::msg::TrackingEvent event; event.header = output.header;
    event.graph_revision = last_graph_revision_; event.map_id = frame.map_id;
    event.type = frame.tracking_state == orb_slam3_msgs::msg::TrackedFrame::OK
      ? orb_slam3_msgs::msg::TrackingEvent::RELOCALIZED
      : orb_slam3_msgs::msg::TrackingEvent::LOST;
    event.detail = "tracking state transition; loop closure not asserted";
    events_pub_->publish(event); last_event_ = event; ++event_publish_count_;
  } else if (last_tracking_state_ == -1 && frame.tracking_state == orb_slam3_msgs::msg::TrackedFrame::OK) {
    orb_slam3_msgs::msg::TrackingEvent event; event.header = output.header;
    event.type = orb_slam3_msgs::msg::TrackingEvent::INITIALIZED; event.map_id = frame.map_id;
    event.detail = "first valid ORB pose"; events_pub_->publish(event); last_event_ = event; ++event_publish_count_;
  }
  last_tracking_state_ = frame.tracking_state;

  if (tracking_image_rate_hz_ > 0.0 && (last_tracking_image_time_.nanoseconds() == 0 ||
      (stamp - last_tracking_image_time_).seconds() >= 1.0 / tracking_image_rate_hz_)) {
    if (image_worker_->submit(mono8(left), output.header)) last_tracking_image_time_ = stamp;
  }
}

void WrapperNode::publishGraph(const ORB_SLAM3::GraphSnapshot& graph, const std_msgs::msg::Header& header) {
  orb_slam3_msgs::msg::GraphSnapshot output; output.header = header; output.revision = graph.revision;
  output.active_map_id = graph.active_map_id; output.active_map_connected = graph.active_map_connected;
  nav_msgs::msg::Path path; path.header = header;
  std::set<std::pair<std::uint64_t, std::uint64_t>> edges;
  std::unordered_map<std::uint64_t, geometry_msgs::msg::Point> points;
  visualization_msgs::msg::MarkerArray keyframe_markers, loop_markers;
  visualization_msgs::msg::Marker keyframe_marker; keyframe_marker.header = header;
  keyframe_marker.ns = "keyframes"; keyframe_marker.id = 0;
  keyframe_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  keyframe_marker.action = visualization_msgs::msg::Marker::ADD;
  keyframe_marker.scale.x = keyframe_marker.scale.y = keyframe_marker.scale.z = 0.08;
  keyframe_marker.color.a = 1.0; keyframe_marker.color.g = 1.0;
  visualization_msgs::msg::Marker loop_marker; loop_marker.header = header;
  loop_marker.ns = "loop_edges"; loop_marker.id = 0;
  loop_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  loop_marker.action = visualization_msgs::msg::Marker::ADD;
  loop_marker.scale.x = 0.025; loop_marker.color.a = 1.0; loop_marker.color.r = 1.0;
  for (const auto& keyframe : graph.keyframes) {
    orb_slam3_msgs::msg::KeyframePose key; key.id = keyframe.id; key.map_id = keyframe.map_id;
    key.bad = keyframe.bad; key.has_parent = keyframe.has_parent; key.parent_id = keyframe.parent_id;
    if (converter_ && converter_->initialized()) key.pose = poseMsg(converter_->toBasePose(keyframe.T_world_camera));
    keyframe_marker.points.push_back(key.pose.position); points[keyframe.id] = key.pose.position;
    output.keyframes.push_back(key);
    geometry_msgs::msg::PoseStamped pose; pose.header = header; pose.pose = key.pose; path.poses.push_back(pose);
  }
  for (const auto& keyframe : graph.keyframes) {
    for (auto id : keyframe.loop_edge_ids) {
      const auto a = std::min(keyframe.id, id), b = std::max(keyframe.id, id);
      if (edges.emplace(a, b).second) {
        orb_slam3_msgs::msg::LoopEdge edge; edge.map_id = keyframe.map_id;
        edge.from_id = a; edge.to_id = b; output.loop_edges.push_back(edge);
        if (points.count(a) && points.count(b)) { loop_marker.points.push_back(points[a]); loop_marker.points.push_back(points[b]); }
      }
    }
  }
  graph_pub_->publish(output); path_pub_->publish(path); last_graph_ = output; ++graph_publish_count_;
  keyframe_markers.markers.push_back(keyframe_marker); loop_markers.markers.push_back(loop_marker);
  keyframes_pub_->publish(keyframe_markers); loops_pub_->publish(loop_markers);
  const auto evidence = classifyGraphDeltaEvidence(previous_graph_, graph);
  RCLCPP_INFO(get_logger(),
              "graph_observation stage=changed revision=%lu raw_loop_edges=%zu previous_baseline=%s extracted_loop_edges=%zu",
              static_cast<unsigned long>(graph.revision), canonicalLoopEdgeCount(graph),
              previous_graph_ ? "true" : "false",
              evidence.loop_edges.size());
  for (const auto type : evidence.event_types) {
    if (type == orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED) continue;
    orb_slam3_msgs::msg::TrackingEvent event; event.header = header; event.type = type;
    event.graph_revision = graph.revision; event.map_id = graph.active_map_id;
    event.detail = "semantic graph evidence observed in snapshot delta";
    events_pub_->publish(event); last_event_ = event; ++event_publish_count_;
  }
  for (const auto& loop_edge : evidence.loop_edges) {
    orb_slam3_msgs::msg::TrackingEvent event; event.header = header;
    event.type = orb_slam3_msgs::msg::TrackingEvent::LOOP_CLOSED;
    event.graph_revision = graph.revision; event.map_id = graph.active_map_id;
    event.detail = "loop_edge=" + std::to_string(loop_edge.first_keyframe_id) + "-" + std::to_string(loop_edge.second_keyframe_id) +
                   " classification=" + loop_edge.classification +
                   " maps=" + std::to_string(loop_edge.first_map_id) + "," + std::to_string(loop_edge.second_map_id) +
                   " active_map=" + std::to_string(loop_edge.active_map_id);
    events_pub_->publish(event); last_event_ = event; ++event_publish_count_;
  }
  previous_graph_ = graph;
  last_graph_revision_ = graph.revision; last_tracked_.graph_revision = graph.revision;
}

void WrapperNode::publishDiagnostics(const std::string& level, const std::string& message) {
  diagnostic_msgs::msg::DiagnosticArray array; diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_name(); status.message = message; status.level = level == "ERROR" ? 2 : 1;
  array.status.push_back(status); diagnostics_pub_->publish(array);
}

}  // namespace orb_slam3_wrapper
