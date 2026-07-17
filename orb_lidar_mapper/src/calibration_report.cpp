#include "orb_lidar_mapper/calibration_report.hpp"

#include <chrono>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cerrno>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "orb_lidar_mapper/calibration_deskew.hpp"

namespace orb_lidar_mapper {
namespace {

std::string jsonEscape(const std::string& value) {
  std::string result;
  for (const char character : value) {
    switch (character) {
      case '\\': result += "\\\\"; break;
      case '"': result += "\\\""; break;
      case '\n': result += "\\n"; break;
      case '\r': result += "\\r"; break;
      case '\t': result += "\\t"; break;
      case '<': result += "\\u003c"; break;
      default: result += character; break;
    }
  }
  return result;
}

const char* methodName(DeskewMethod method) {
  switch (method) {
    case DeskewMethod::kOdom: return "Odom";
    case DeskewMethod::kImu: return "IMU";
    case DeskewMethod::kExistingScan: return "Existing /scan";
  }
  return "Unknown";
}

const char* resultName(ResultClass result) {
  switch (result) {
    case ResultClass::kConsistent: return "CONSISTENT";
    case ResultClass::kLikelyOffsetError: return "LIKELY_OFFSET_ERROR";
    case ResultClass::kInconclusive: return "INCONCLUSIVE";
  }
  return "INCONCLUSIVE";
}

std::string number(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

std::string csvNumber(double value) {
  if (!std::isfinite(value)) return "";
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << value;
  return stream.str();
}

std::string csvField(const std::string& value) {
  std::string escaped = "\"";
  for (const char character : value) {
    if (character == '"') escaped += "\"\"";
    else escaped += character;
  }
  escaped += '"';
  return escaped;
}

Point2 transformLocalToWorld(const Pose2& world, Point2 local) {
  const double c = std::cos(world.yaw);
  const double s = std::sin(world.yaw);
  return {world.x + c * local.x - s * local.y,
          world.y + s * local.x + c * local.y};
}

// Build a sparse map preview by odom-deskewing raw scans at the candidate
// mount offset, then transforming the midpoint cloud into the world frame.
// Using undeskewed rays at a single scan stamp produces ghost walls under spin.
std::vector<Point2> mapPoints(const RotationDataset& dataset, double offset_m) {
  std::vector<Point2> points;
  if (!std::isfinite(offset_m) || dataset.odom_poses.empty() ||
      dataset.raw_scans.empty()) {
    return points;
  }
  const auto retention = dataset.odom_poses.back().stamp_ns -
                         dataset.odom_poses.front().stamp_ns + 1'000'000'000LL;
  TimedPoseBuffer odom(std::max<std::int64_t>(retention, 1'000'000'000LL),
                       1'000'000'000LL);
  for (const auto& pose : dataset.odom_poses) odom.push(pose);

  const StaticLidarMount mount{offset_m, 0.0, 0.0, kPi};
  const double range_cap_m = 12.0;
  const std::size_t stride =
      std::max<std::size_t>(1, dataset.raw_scans.size() / 80);
  for (std::size_t scan_index = 0; scan_index < dataset.raw_scans.size();
       scan_index += stride) {
    const auto& scan = dataset.raw_scans[scan_index];
    const auto deskewed = deskewWithOdom(scan, odom, mount, range_cap_m);
    if (!deskewed || deskewed->points.empty()) continue;
    const auto pose = odom.interpolate(deskewed->reference_stamp_ns);
    if (!pose) continue;
    const Pose2 world = *pose * Pose2{offset_m, 0.0, kPi};
    const std::size_t ray_stride =
        std::max<std::size_t>(1, deskewed->points.size() / 180);
    for (std::size_t i = 0; i < deskewed->points.size(); i += ray_stride) {
      points.push_back(transformLocalToWorld(world, deskewed->points[i]));
    }
  }
  return points;
}

struct IcpMapEdge {
  std::uint64_t source_id{};
  std::uint64_t target_id{};
  Pose2 source_to_target;
};

struct IcpMapResult {
  std::vector<Point2> points;
  std::size_t scan_count{};
  std::size_t edge_count{};
  std::uint64_t root_scan_id{};
};

// Stack odom-deskewed scans using a BFS pose chain of accepted Odom ICP pairs.
// Independent of the pure-odom recorded/estimated map stacks.
IcpMapResult icpMapPoints(const CalibrationRun& run) {
  IcpMapResult result;
  const auto& dataset = run.dataset;
  if (dataset.odom_poses.empty() || dataset.raw_scans.empty()) return result;

  std::unordered_map<std::uint64_t, const ScanValue*> scans_by_id;
  scans_by_id.reserve(dataset.raw_scans.size());
  for (const auto& scan : dataset.raw_scans) {
    scans_by_id[scan.id] = &scan;
  }

  const auto retention = dataset.odom_poses.back().stamp_ns -
                         dataset.odom_poses.front().stamp_ns + 1'000'000'000LL;
  TimedPoseBuffer odom(std::max<std::int64_t>(retention, 1'000'000'000LL),
                       1'000'000'000LL);
  for (const auto& pose : dataset.odom_poses) odom.push(pose);

  const StaticLidarMount mount{dataset.recorded_mount.x_m, 0.0, 0.0, kPi};
  const double range_cap_m = 12.0;

  std::unordered_map<std::uint64_t, std::vector<Point2>> local_points;
  auto ensureDeskewed = [&](std::uint64_t scan_id) -> bool {
    if (local_points.count(scan_id)) return !local_points[scan_id].empty();
    const auto it = scans_by_id.find(scan_id);
    if (it == scans_by_id.end()) {
      local_points[scan_id] = {};
      return false;
    }
    const auto deskewed = deskewWithOdom(*it->second, odom, mount, range_cap_m);
    if (!deskewed || deskewed->points.empty()) {
      local_points[scan_id] = {};
      return false;
    }
    local_points[scan_id] = deskewed->points;
    return true;
  };

  std::vector<IcpMapEdge> edges;
  for (const auto& sample : run.center_samples) {
    if (!sample.accepted || sample.method != DeskewMethod::kOdom) continue;
    if (sample.source_scan_id == sample.target_scan_id) continue;
    if (!ensureDeskewed(sample.source_scan_id) ||
        !ensureDeskewed(sample.target_scan_id)) {
      continue;
    }
    edges.push_back(
        {sample.source_scan_id, sample.target_scan_id, sample.icp.source_to_target});
  }
  if (edges.empty()) return result;

  // Undirected adjacency for connected components (edge indices).
  std::map<std::uint64_t, std::vector<std::size_t>> adjacency;
  for (std::size_t i = 0; i < edges.size(); ++i) {
    adjacency[edges[i].source_id].push_back(i);
    adjacency[edges[i].target_id].push_back(i);
  }

  std::unordered_set<std::uint64_t> visited;
  std::vector<std::uint64_t> best_nodes;
  std::size_t best_edge_count = 0;
  std::uint64_t best_root = 0;

  for (const auto& [seed, _] : adjacency) {
    if (visited.count(seed)) continue;
    std::vector<std::uint64_t> component_nodes;
    std::unordered_set<std::size_t> component_edges;
    std::deque<std::uint64_t> queue;
    queue.push_back(seed);
    visited.insert(seed);
    while (!queue.empty()) {
      const auto node = queue.front();
      queue.pop_front();
      component_nodes.push_back(node);
      for (const auto edge_index : adjacency[node]) {
        component_edges.insert(edge_index);
        const auto& edge = edges[edge_index];
        const auto other =
            edge.source_id == node ? edge.target_id : edge.source_id;
        if (visited.insert(other).second) queue.push_back(other);
      }
    }
    std::sort(component_nodes.begin(), component_nodes.end());
    const auto root = component_nodes.front();
    const auto edge_count = component_edges.size();
    const bool better =
        edge_count > best_edge_count ||
        (edge_count == best_edge_count &&
         (component_nodes.size() > best_nodes.size() ||
          (component_nodes.size() == best_nodes.size() &&
           (best_nodes.empty() || root < best_root))));
    if (better) {
      best_edge_count = edge_count;
      best_nodes = std::move(component_nodes);
      best_root = root;
    }
  }
  if (best_nodes.size() < 2 || best_edge_count == 0) return result;

  // BFS poses from smallest scan id in the largest component.
  std::unordered_map<std::uint64_t, Pose2> poses;
  poses[best_root] = {};
  std::deque<std::uint64_t> queue;
  queue.push_back(best_root);
  std::unordered_set<std::uint64_t> placed{best_root};
  while (!queue.empty()) {
    const auto node = queue.front();
    queue.pop_front();
    for (const auto edge_index : adjacency[node]) {
      const auto& edge = edges[edge_index];
      // Only traverse edges whose both endpoints are in the chosen component.
      if (std::find(best_nodes.begin(), best_nodes.end(), edge.source_id) ==
              best_nodes.end() ||
          std::find(best_nodes.begin(), best_nodes.end(), edge.target_id) ==
              best_nodes.end()) {
        continue;
      }
      const Pose2& T = edge.source_to_target;
      if (poses.count(edge.target_id) && !poses.count(edge.source_id)) {
        // pose[source] = pose[target] * T
        poses[edge.source_id] = poses[edge.target_id] * T;
        if (placed.insert(edge.source_id).second) queue.push_back(edge.source_id);
      } else if (poses.count(edge.source_id) && !poses.count(edge.target_id)) {
        // pose[target] = pose[source] * T.inverse()
        poses[edge.target_id] = poses[edge.source_id] * T.inverse();
        if (placed.insert(edge.target_id).second) queue.push_back(edge.target_id);
      }
    }
  }

  // Subsample scans (~80) and rays (~180) like mapPoints for HTML size.
  std::vector<std::uint64_t> ordered_nodes = best_nodes;
  std::sort(ordered_nodes.begin(), ordered_nodes.end());
  const std::size_t scan_stride =
      std::max<std::size_t>(1, ordered_nodes.size() / 80);
  for (std::size_t scan_index = 0; scan_index < ordered_nodes.size();
       scan_index += scan_stride) {
    const auto scan_id = ordered_nodes[scan_index];
    const auto pose_it = poses.find(scan_id);
    const auto pts_it = local_points.find(scan_id);
    if (pose_it == poses.end() || pts_it == local_points.end() ||
        pts_it->second.empty()) {
      continue;
    }
    const std::size_t ray_stride =
        std::max<std::size_t>(1, pts_it->second.size() / 180);
    for (std::size_t i = 0; i < pts_it->second.size(); i += ray_stride) {
      result.points.push_back(
          transformLocalToWorld(pose_it->second, pts_it->second[i]));
    }
  }

  result.scan_count = poses.size();
  result.edge_count = best_edge_count;
  result.root_scan_id = best_root;
  return result;
}

std::string jsonDocument(const CalibrationRun& run) {
  std::ostringstream json;
  json << "{\"configuration\":{";
  json << "\"bag\":\"" << jsonEscape(run.config.bag_path.string()) << "\",";
  json << "\"output\":\"" << jsonEscape(run.config.output_dir.string()) << "\",";
  json << "\"overwrite\":" << (run.config.overwrite ? "true" : "false") << ",";
  json << "\"min_abs_omega\":" << number(run.config.min_abs_omega) << ",";
  json << "\"max_abs_omega\":" << number(run.config.max_abs_omega) << ",";
  json << "\"max_abs_linear_speed\":" << number(run.config.max_abs_linear_speed) << ",";
  json << "\"range_cap_m\":" << number(run.config.range_cap_m) << "},";
  json << "\"recorded_mount\":{";
  json << "\"x_m\":" << number(run.dataset.recorded_mount.x_m) << ",";
  json << "\"y_m\":" << number(run.dataset.recorded_mount.y_m) << ",";
  json << "\"z_m\":" << number(run.dataset.recorded_mount.z_m) << ",";
  json << "\"yaw_rad\":" << number(run.dataset.recorded_mount.yaw_rad) << "},";
  json << "\"dataset\":{";
  json << "\"raw_scans\":" << run.dataset.raw_scans.size() << ",";
  json << "\"undistorted_scans\":" << run.dataset.undistorted_scans.size() << ",";
  json << "\"odom_poses\":" << run.dataset.odom_poses.size() << ",";
  json << "\"imu_yaw_rates\":" << run.dataset.imu_yaw_rates.size() << "},";
  const auto recorded_map = mapPoints(run.dataset, run.dataset.recorded_mount.x_m);
  const auto estimated_map = mapPoints(run.dataset, run.aggregate.consensus_offset_m);
  const auto icp_map = icpMapPoints(run);
  json << "\"maps\":{" << "\"recorded\":[";
  for (std::size_t index = 0; index < recorded_map.size(); ++index) {
    if (index != 0) json << ',';
    json << '[' << number(recorded_map[index].x) << ',' << number(recorded_map[index].y) << ']';
  }
  json << "],\"estimated\":[";
  for (std::size_t index = 0; index < estimated_map.size(); ++index) {
    if (index != 0) json << ',';
    json << '[' << number(estimated_map[index].x) << ',' << number(estimated_map[index].y) << ']';
  }
  json << "],\"icp\":[";
  for (std::size_t index = 0; index < icp_map.points.size(); ++index) {
    if (index != 0) json << ',';
    json << '[' << number(icp_map.points[index].x) << ','
         << number(icp_map.points[index].y) << ']';
  }
  json << "]";
  if (!icp_map.points.empty()) {
    json << ",\"icp_meta\":{"
         << "\"scan_count\":" << icp_map.scan_count << ","
         << "\"edge_count\":" << icp_map.edge_count << ","
         << "\"root_scan_id\":" << icp_map.root_scan_id << "}";
  }
  json << "},";
  json << "\"center_samples\":[";
  for (std::size_t index = 0; index < run.center_samples.size(); ++index) {
    if (index != 0) json << ',';
    const auto& sample = run.center_samples[index];
    json << "{\"method\":\"" << methodName(sample.method) << "\",";
    json << "\"source_scan_id\":" << sample.source_scan_id << ",\"target_scan_id\":" << sample.target_scan_id << ",";
    json << "\"yaw_sector\":" << sample.yaw_sector << ",\"accepted\":" << (sample.accepted ? "true" : "false") << ",";
    json << "\"center_x_m\":" << number(sample.center.x) << ",\"center_y_m\":" << number(sample.center.y) << ",";
    json << "\"rejection_reason\":\"" << jsonEscape(sample.rejection_reason) << "\"}";
  }
  json << "],";
  json << "\"methods\":[";
  for (std::size_t index = 0; index < run.methods.size(); ++index) {
    if (index != 0) json << ',';
    const auto& method = run.methods[index];
    json << "{\"method\":\"" << methodName(method.method) << "\",";
    json << "\"reliable\":" << (method.reliable ? "true" : "false") << ",";
    json << "\"center_x_m\":" << number(method.center_x_m) << ",\"center_y_m\":" << number(method.center_y_m) << ",";
    json << "\"forward_offset_m\":" << number(method.forward_offset_m) << ",";
    json << "\"delta_from_recorded_m\":" << number(method.forward_offset_m - run.dataset.recorded_mount.x_m) << ",";
    json << "\"confidence_95_m\":{" << "\"low_m\":" << number(method.confidence_95_m.low_m)
         << ",\"high_m\":" << number(method.confidence_95_m.high_m) << "},";
    json << "\"accepted_pairs\":" << method.accepted_pairs << ",\"attempted_pairs\":" << method.attempted_pairs << ",";
    json << "\"covered_yaw_sectors\":" << method.covered_yaw_sectors << ",\"median_rmse_m\":" << number(method.median_rmse_m)
         << ",\"median_overlap\":" << number(method.median_overlap) << ",\"rejection_counts\":{";
    bool first_reason = true;
    for (const auto& [reason, count] : method.rejection_counts) {
      if (!first_reason) json << ',';
      first_reason = false;
      json << "\"" << jsonEscape(reason) << "\":" << count;
    }
    json << "}}";
  }
  json << "],\"aggregate\":{";
  json << "\"classification\":\"" << resultName(run.aggregate.classification) << "\",";
  json << "\"consensus_offset_m\":" << number(run.aggregate.consensus_offset_m) << ",";
  json << "\"confidence_95_m\":{" << "\"low_m\":" << number(run.aggregate.confidence_95_m.low_m)
       << ",\"high_m\":" << number(run.aggregate.confidence_95_m.high_m) << "},";
  json << "\"reason\":\"" << jsonEscape(run.aggregate.reason) << "\"},";
  json << "\"sharpness\":{" << "\"reliable\":" << (run.sharpness.reliable ? "true" : "false") << ",";
  json << "\"best_offset_m\":" << number(run.sharpness.best_offset_m) << ",\"rejection_reason\":\""
       << jsonEscape(run.sharpness.rejection_reason) << "\",\"coarse\":[";
  for (std::size_t index = 0; index < run.sharpness.coarse.size(); ++index) {
    if (index != 0) json << ',';
    json << "{\"offset_m\":" << number(run.sharpness.coarse[index].offset_m)
         << ",\"score\":" << number(run.sharpness.coarse[index].score) << '}';
  }
  json << "],\"refined\":[";
  for (std::size_t index = 0; index < run.sharpness.refined.size(); ++index) {
    if (index != 0) json << ',';
    json << "{\"offset_m\":" << number(run.sharpness.refined[index].offset_m)
         << ",\"score\":" << number(run.sharpness.refined[index].score) << '}';
  }
  json << "]}";
  json << "}";
  return json.str();
}

std::string centersCsv(const CalibrationRun& run) {
  std::ostringstream csv;
  csv << "method,source_scan_id,target_scan_id,yaw_sector,accepted,center_x_m,center_y_m,forward_offset_m,delta_from_recorded_m,trimmed_rmse_m,overlap_ratio,rejection_reason\n";
  for (const auto& sample : run.center_samples) {
    csv << csvField(methodName(sample.method)) << ',' << sample.source_scan_id << ',' << sample.target_scan_id << ','
        << sample.yaw_sector << ',' << (sample.accepted ? "true" : "false") << ','
        << number(sample.center.x) << ',' << number(sample.center.y) << ',' << number(sample.center.x) << ','
        << number(sample.center.x - run.dataset.recorded_mount.x_m) << ','
        << csvNumber(sample.icp.trimmed_rmse_m) << ',' << csvNumber(sample.icp.overlap_ratio) << ','
        << csvField(sample.rejection_reason) << "\n";
  }
  return csv.str();
}

std::string sharpnessCsv(const CalibrationRun& run) {
  std::ostringstream csv;
  csv << "grid,offset_m,score\n";
  for (const auto& point : run.sharpness.coarse) csv << "coarse," << csvNumber(point.offset_m) << ',' << csvNumber(point.score) << "\n";
  for (const auto& point : run.sharpness.refined) csv << "refined," << csvNumber(point.offset_m) << ',' << csvNumber(point.score) << "\n";
  return csv.str();
}

std::string reportHtml(const CalibrationRun& run) {
  const auto json = jsonDocument(run);
  std::ostringstream html;
  html << R"HTML(<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lidar rotation-center calibration</title><style>
:root{font-family:system-ui,sans-serif;color:#17202a;background:#f5f7fa}body{margin:0;padding:20px}main{max-width:1180px;margin:auto}section{background:#fff;border:1px solid #d9e0e8;border-radius:12px;padding:16px;margin:12px 0;box-shadow:0 1px 3px #10203018}h1{margin-top:0}.status{font-size:1.3rem;font-weight:700}.warning{color:#9b2c2c;background:#fff4f4;padding:10px;border-radius:6px}.legend{display:flex;flex-wrap:wrap;gap:10px;font-size:.85rem}.legend span{white-space:nowrap}.swatch{display:inline-block;width:.8em;height:.8em;border-radius:50%;margin-right:4px}table{border-collapse:collapse;width:100%;font-size:.9rem}th,td{padding:7px;border-bottom:1px solid #e5e9ef;text-align:left}canvas{display:block;width:100%;height:auto;max-width:900px;margin:8px auto;border:1px solid #d9e0e8;background:#fff} .maps{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px}.map-card{min-width:0}@media(max-width:900px){table{display:block;overflow-x:auto;white-space:nowrap}}@media(max-width:600px){body{padding:8px}.maps{grid-template-columns:1fr}section{padding:10px}table{display:block;width:100%;overflow:visible;white-space:normal;font-size:.75rem}thead{position:absolute;width:1px;height:1px;overflow:hidden;clip:rect(0 0 0 0);white-space:nowrap}tbody,tr,td{display:block;width:100%;box-sizing:border-box}tr{border-bottom:1px solid #d9e0e8;padding:6px 0}td{display:grid;grid-template-columns:minmax(0,42%) minmax(0,1fr);gap:8px;padding:6px 0;border-bottom:0;min-width:0;white-space:normal;overflow-wrap:anywhere}td::before{content:attr(data-label);font-weight:650;color:#334155}}
</style></head><body><main><h1>Lidar rotation-center calibration</h1>
<section><div class="status" id="classification"></div><div id="warning"></div><p>Recorded offset: <span id="recorded"></span> m; estimated offset: <span id="estimated"></span> m</p></section>
<section><h2>Raw center scatter</h2><p>Accepted samples use method colors; rejected samples are outlined in red. Recorded center is gold.</p><div class="legend" id="center-legend"><span><i class="swatch" style="background:#2563eb"></i>Odom</span><span><i class="swatch" style="background:#16a34a"></i>IMU</span><span><i class="swatch" style="background:#9333ea"></i>Existing /scan</span><span><i class="swatch" style="background:#d4a017"></i>Recorded center</span></div><canvas id="center-scatter" width="900" height="360" aria-label="Raw center scatter"></canvas></section>
<section><h2>Method estimates</h2><table><thead><tr><th>Method</th><th>Center x</th><th>Center y</th><th>Forward offset</th><th>Delta</th><th>95% CI</th><th>Accepted/attempted</th><th>Sectors</th><th>RMSE</th><th>Overlap</th></tr></thead><tbody id="methods"></tbody></table></section>
<section><h2>Sharpness curve</h2><canvas id="sharpness" width="900" height="360" aria-label="Map sharpness curve"></canvas></section>
<section><h2>Map views</h2><div class="maps"><div class="map-card"><h3>Recorded map view</h3><canvas id="recorded-map" width="600" height="360" aria-label="Recorded map view"></canvas></div><div class="map-card"><h3>Estimated map view</h3><canvas id="estimated-map" width="600" height="360" aria-label="Estimated map view"></canvas></div><div class="map-card"><h3>ICP map view (Odom accepted pairs)</h3><canvas id="icp-map" width="600" height="360" aria-label="ICP map view"></canvas></div></div></section>
<script>const calibration=)HTML" << json << R"HTML(;
const finite=v=>typeof v==='number'&&Number.isFinite(v);const fixed=v=>finite(v)?v.toFixed(3):'—';const recorded=calibration.recorded_mount.x_m;document.getElementById('classification').textContent=calibration.aggregate.classification;document.getElementById('recorded').textContent=fixed(recorded);document.getElementById('estimated').textContent=fixed(calibration.aggregate.consensus_offset_m);document.getElementById('warning').textContent=calibration.aggregate.reason||calibration.sharpness.rejection_reason;
const methods=document.getElementById('methods');const methodColumns=['Method','Center x','Center y','Forward offset','Delta','95% CI','Accepted/attempted','Sectors','RMSE','Overlap'];calibration.methods.forEach(m=>{const row=document.createElement('tr');[m.method,fixed(m.center_x_m),fixed(m.center_y_m),fixed(m.forward_offset_m),fixed(m.delta_from_recorded_m),`[${fixed(m.confidence_95_m.low_m)}, ${fixed(m.confidence_95_m.high_m)}]`,`${m.accepted_pairs}/${m.attempted_pairs}`,m.covered_yaw_sectors,fixed(m.median_rmse_m),fixed(m.median_overlap)].forEach((v,index)=>{const cell=document.createElement('td');cell.dataset.label=methodColumns[index];cell.textContent=v;row.appendChild(cell)});methods.appendChild(row)});
const methodColors={'Odom':'#2563eb','IMU':'#16a34a','Existing /scan':'#9333ea'};function context(id){const c=document.getElementById(id),x=c.getContext('2d');x.clearRect(0,0,c.width,c.height);x.strokeStyle='#cbd5e1';x.strokeRect(0,0,c.width,c.height);return [c,x]};function plotCenters(){const [c,x]=context('center-scatter');const sx=v=>50+(v+0.05)*700,sy=v=>310-(v+0.30)*700;x.strokeStyle='#94a3b8';x.beginPath();x.moveTo(sx(0),20);x.lineTo(sx(0),340);x.moveTo(40,sy(0));x.lineTo(860,sy(0));x.stroke();calibration.center_samples.forEach(s=>{if(!finite(s.center_x_m)||!finite(s.center_y_m))return;const px=sx(s.center_x_m-recorded),py=sy(s.center_y_m);x.beginPath();x.arc(px,py,s.accepted?4:5,0,Math.PI*2);if(s.accepted){x.fillStyle=methodColors[s.method]||'#475569';x.fill()}else{x.strokeStyle='#dc2626';x.lineWidth=2;x.stroke();x.lineWidth=1}});if(finite(recorded)){x.fillStyle='#d4a017';x.beginPath();x.arc(sx(0),sy(0),7,0,Math.PI*2);x.fill()}};function drawSeries(x,series,stroke,lo,hi,max){const points=series.filter(q=>finite(q.offset_m)&&finite(q.score));if(!points.length)return;x.strokeStyle=stroke;x.beginPath();points.forEach((q,i)=>{const px=40+(q.offset_m-lo)/(hi-lo||1)*820,py=330-q.score/(max||1)*290;if(i===0)x.moveTo(px,py);else x.lineTo(px,py)});x.stroke()};function plotSharpness(){const [c,x]=context('sharpness');const p=calibration.sharpness.coarse.concat(calibration.sharpness.refined).filter(q=>finite(q.offset_m)&&finite(q.score));if(!p.length)return;const lo=Math.min(...p.map(q=>q.offset_m)),hi=Math.max(...p.map(q=>q.offset_m)),max=Math.max(...p.map(q=>q.score),1);drawSeries(x,calibration.sharpness.coarse,'#0f766e',lo,hi,max);drawSeries(x,calibration.sharpness.refined,'#f97316',lo,hi,max);if(finite(calibration.sharpness.best_offset_m)){x.fillStyle='#b91c1c';x.beginPath();x.arc(40+(calibration.sharpness.best_offset_m-lo)/(hi-lo||1)*820,330-(Math.min(...p.map(q=>q.score))/(max||1))*290,5,0,Math.PI*2);x.fill()}};function plotMap(id,key){const [c,x]=context(id),p=calibration.maps[key].filter(q=>Array.isArray(q)&&finite(q[0])&&finite(q[1]));if(!p.length)return;const loX=Math.min(...p.map(q=>q[0])),hiX=Math.max(...p.map(q=>q[0])),loY=Math.min(...p.map(q=>q[1])),hiY=Math.max(...p.map(q=>q[1]));const sx=v=>20+(v-loX)/(hiX-loX||1)*560,sy=v=>340-(v-loY)/(hiY-loY||1)*320;x.fillStyle='#475569';p.forEach(q=>{x.beginPath();x.arc(sx(q[0]),sy(q[1]),1.5,0,Math.PI*2);x.fill()})};plotCenters();plotSharpness();plotMap('recorded-map','recorded');plotMap('estimated-map','estimated');plotMap('icp-map','icp');</script></main></body></html>)HTML";
  return html.str();
}

struct PendingOutput {
  std::filesystem::path final_path;
  std::filesystem::path temporary_path;
  std::filesystem::path backup_path;
  bool had_original{};
  bool published{};
};

std::filesystem::path uniqueSibling(const std::filesystem::path& path, const char* tag) {
  static std::atomic<std::uint64_t> counter{0};
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  for (std::uint64_t attempt = 0; attempt < 100; ++attempt) {
    const auto suffix = std::string(".") + tag + "." + std::to_string(static_cast<long long>(::getpid())) +
      "." + std::to_string(stamp) + "." +
      std::to_string(counter.fetch_add(1) + attempt);
    const auto candidate = path.string() + suffix;
    const int fd = ::open(candidate.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (fd >= 0) {
      ::close(fd);
      return candidate;
    }
    if (errno != EEXIST) throw std::runtime_error("cannot reserve temporary output " + candidate);
  }
  throw std::runtime_error("cannot reserve unique temporary output " + path.string());
}

void removeIfExists(const std::filesystem::path& path) {
  std::error_code error;
  std::filesystem::remove(path, error);
}

void writeTemporary(const std::filesystem::path& path, const std::string& content) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) throw std::runtime_error("cannot open temporary output " + path.string());
  output.write(content.data(), static_cast<std::streamsize>(content.size()));
  output.flush();
  if (!output) throw std::runtime_error("cannot finish temporary output " + path.string());
}

void publishReportSet(const std::vector<std::pair<std::filesystem::path, std::string>>& files,
                      bool overwrite) {
  std::vector<PendingOutput> pending;
  pending.reserve(files.size());
  try {
    for (const auto& [final_path, content] : files) {
      PendingOutput item;
      item.final_path = final_path;
      item.temporary_path = uniqueSibling(final_path, "tmp");
      pending.push_back(std::move(item));
      const char* write_failure = std::getenv("TASK5_REPORT_FAIL_WRITE_INDEX");
      if (write_failure != nullptr && pending.size() - 1U == static_cast<std::size_t>(std::stoul(write_failure))) {
        throw std::runtime_error("injected report write failure");
      }
      writeTemporary(pending.back().temporary_path, content);
    }
    for (auto& item : pending) {
      std::error_code exists_error;
      item.had_original = std::filesystem::exists(item.final_path, exists_error);
      if (exists_error) throw std::runtime_error("cannot inspect output " + item.final_path.string());
      if (item.had_original) {
        if (!overwrite) throw std::runtime_error("output file exists; use --overwrite");
        item.backup_path = uniqueSibling(item.final_path, "bak");
        std::filesystem::rename(item.final_path, item.backup_path);
      }
    }
    const char* failure = std::getenv("TASK5_REPORT_FAIL_PUBLISH_INDEX");
    const auto failure_index = failure == nullptr ? files.size() : static_cast<std::size_t>(std::stoul(failure));
    for (std::size_t index = 0; index < pending.size(); ++index) {
      if (index == failure_index) throw std::runtime_error("injected report publication failure");
      std::filesystem::rename(pending[index].temporary_path, pending[index].final_path);
      pending[index].published = true;
    }
    for (const auto& item : pending) removeIfExists(item.backup_path);
  } catch (...) {
    for (auto& item : pending) {
      if (item.published) removeIfExists(item.final_path);
      removeIfExists(item.temporary_path);
    }
    for (auto& item : pending) {
      if (item.had_original && std::filesystem::exists(item.backup_path)) {
        std::error_code restore_error;
        std::filesystem::rename(item.backup_path, item.final_path, restore_error);
        if (restore_error) removeIfExists(item.backup_path);
      } else {
        removeIfExists(item.backup_path);
      }
    }
    throw;
  }
}

}  // namespace

void writeCalibrationReport(const CalibrationRun& run) {
  const auto& output = run.config.output_dir;
  std::error_code error;
  if (std::filesystem::exists(output, error)) {
    if (!std::filesystem::is_directory(output, error)) throw std::runtime_error("output path is not a directory");
    if (!run.config.overwrite) throw std::runtime_error("output directory exists; use --overwrite");
  } else {
    if (!std::filesystem::create_directories(output, error) || error) {
      throw std::runtime_error("cannot create output directory: " + error.message());
    }
  }
  publishReportSet({
    {output / "calibration.json", jsonDocument(run)},
    {output / "centers.csv", centersCsv(run)},
    {output / "sharpness.csv", sharpnessCsv(run)},
    {output / "report.html", reportHtml(run)}}, run.config.overwrite);
}

}  // namespace orb_lidar_mapper
