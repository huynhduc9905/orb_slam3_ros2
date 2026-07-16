#include "orb_lidar_mapper/calibration_report.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

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
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

std::string csvNumber(double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << value;
  return stream.str();
}

std::vector<Point2> mapPoints(const RotationDataset& dataset, double offset_m) {
  std::vector<Point2> points;
  if (dataset.odom_poses.empty()) return points;
  const auto retention = dataset.odom_poses.back().stamp_ns - dataset.odom_poses.front().stamp_ns + 1'000'000'000LL;
  TimedPoseBuffer odom(std::max<std::int64_t>(retention, 1'000'000'000LL), 1'000'000'000LL);
  for (const auto& pose : dataset.odom_poses) odom.push(pose);
  const std::size_t stride = std::max<std::size_t>(1, dataset.raw_scans.size() / 80);
  for (std::size_t scan_index = 0; scan_index < dataset.raw_scans.size(); scan_index += stride) {
    const auto& scan = dataset.raw_scans[scan_index];
    const auto pose = odom.interpolate(scan.stamp_ns);
    if (!pose) continue;
    const Pose2 mount{offset_m, 0.0, kPi};
    const Pose2 world = *pose * mount;
    const std::size_t ray_stride = std::max<std::size_t>(1, scan.ranges.size() / 180);
    for (std::size_t ray = 0; ray < scan.ranges.size(); ray += ray_stride) {
      const double range = scan.ranges[ray];
      if (!std::isfinite(range) || range < std::max(0.15, static_cast<double>(scan.range_min)) ||
          range > static_cast<double>(scan.range_max)) continue;
      const double angle = static_cast<double>(scan.angle_min) +
                           static_cast<double>(ray) * scan.angle_increment;
      points.push_back({world.x + std::cos(world.yaw + angle) * range,
                        world.y + std::sin(world.yaw + angle) * range});
    }
  }
  return points;
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
  json << "]},";
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
    csv << methodName(sample.method) << ',' << sample.source_scan_id << ',' << sample.target_scan_id << ','
        << sample.yaw_sector << ',' << (sample.accepted ? "true" : "false") << ','
        << number(sample.center.x) << ',' << number(sample.center.y) << ',' << number(sample.center.x) << ','
        << number(sample.center.x - run.dataset.recorded_mount.x_m) << ','
        << number(sample.icp.trimmed_rmse_m) << ',' << number(sample.icp.overlap_ratio) << ','
        << '"' << jsonEscape(sample.rejection_reason) << "\"\n";
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
:root{font-family:system-ui,sans-serif;color:#17202a;background:#f5f7fa}body{margin:0;padding:20px}main{max-width:1180px;margin:auto}section{background:#fff;border:1px solid #d9e0e8;border-radius:12px;padding:16px;margin:12px 0;box-shadow:0 1px 3px #10203018}h1{margin-top:0}.status{font-size:1.3rem;font-weight:700}.warning{color:#9b2c2c;background:#fff4f4;padding:10px;border-radius:6px}table{border-collapse:collapse;width:100%;font-size:.9rem}th,td{padding:7px;border-bottom:1px solid #e5e9ef;text-align:left}canvas{display:block;width:100%;height:auto;max-width:900px;margin:8px auto;border:1px solid #d9e0e8;background:#fff} .maps{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px}.map-card{min-width:0}@media(max-width:900px){table{display:block;overflow-x:auto;white-space:nowrap}}@media(max-width:600px){body{padding:8px}.maps{grid-template-columns:1fr}section{padding:10px}table{font-size:.75rem}}
</style></head><body><main><h1>Lidar rotation-center calibration</h1>
<section><div class="status" id="classification"></div><div id="warning"></div><p>Recorded offset: <span id="recorded"></span> m; estimated offset: <span id="estimated"></span> m</p></section>
<section><h2>Raw center scatter</h2><p>Accepted center samples (blue), Rejected center samples (red), Recorded center (gold).</p><canvas id="center-scatter" width="900" height="360" aria-label="Raw center scatter"></canvas></section>
<section><h2>Method estimates</h2><table><thead><tr><th>Method</th><th>Center x</th><th>Center y</th><th>Forward offset</th><th>Delta</th><th>95% CI</th><th>Accepted/attempted</th><th>Sectors</th><th>RMSE</th><th>Overlap</th></tr></thead><tbody id="methods"></tbody></table></section>
<section><h2>Sharpness curve</h2><canvas id="sharpness" width="900" height="360" aria-label="Map sharpness curve"></canvas></section>
<section><h2>Map views</h2><div class="maps"><div class="map-card"><h3>Recorded map view</h3><canvas id="recorded-map" width="600" height="360" aria-label="Recorded map view"></canvas></div><div class="map-card"><h3>Estimated map view</h3><canvas id="estimated-map" width="600" height="360" aria-label="Estimated map view"></canvas></div></div></section>
<script>const calibration=)HTML" << json << R"HTML(;
const recorded=calibration.recorded_mount.x_m;document.getElementById('classification').textContent=calibration.aggregate.classification;document.getElementById('recorded').textContent=recorded.toFixed(3);document.getElementById('estimated').textContent=calibration.aggregate.consensus_offset_m.toFixed(3);document.getElementById('warning').textContent=calibration.aggregate.reason||calibration.sharpness.rejection_reason;
const methods=document.getElementById('methods');calibration.methods.forEach(m=>{const row=document.createElement('tr');[m.method,m.center_x_m.toFixed(3),m.center_y_m.toFixed(3),m.forward_offset_m.toFixed(3),m.delta_from_recorded_m.toFixed(3),`[${m.confidence_95_m.low_m.toFixed(3)}, ${m.confidence_95_m.high_m.toFixed(3)}]`,`${m.accepted_pairs}/${m.attempted_pairs}`,m.covered_yaw_sectors,m.median_rmse_m.toFixed(3),m.median_overlap.toFixed(3)].forEach(v=>{const cell=document.createElement('td');cell.textContent=v;row.appendChild(cell)});methods.appendChild(row)});
function context(id){const c=document.getElementById(id),x=c.getContext('2d');x.clearRect(0,0,c.width,c.height);x.strokeStyle='#cbd5e1';x.strokeRect(0,0,c.width,c.height);return [c,x]};function plotCenters(){const [c,x]=context('center-scatter');const sx=v=>50+(v+0.05)*700,sy=v=>310-(v+0.30)*700;x.strokeStyle='#94a3b8';x.beginPath();x.moveTo(sx(0),20);x.lineTo(sx(0),340);x.moveTo(40,sy(0));x.lineTo(860,sy(0));x.stroke();calibration.center_samples.forEach(s=>{x.fillStyle=s.accepted?'#2563eb':'#dc2626';x.beginPath();x.arc(sx(s.center_x_m-recorded),sy(s.center_y_m),s.accepted?3:4,0,Math.PI*2);x.fill()});x.fillStyle='#d4a017';x.beginPath();x.arc(sx(0),sy(0),7,0,Math.PI*2);x.fill()};function plotSharpness(){const [c,x]=context('sharpness');const p=calibration.sharpness.coarse.concat(calibration.sharpness.refined).filter(q=>Number.isFinite(q.score));if(!p.length)return;const lo=Math.min(...p.map(q=>q.offset_m)),hi=Math.max(...p.map(q=>q.offset_m)),max=Math.max(...p.map(q=>q.score));const sx=v=>40+(v-lo)/(hi-lo)*820,sy=v=>330-v/max*290;x.strokeStyle='#0f766e';x.beginPath();p.forEach((q,i)=>{if(i===0)x.moveTo(sx(q.offset_m),sy(q.score));else x.lineTo(sx(q.offset_m),sy(q.score))});x.stroke();x.fillStyle='#b91c1c';x.beginPath();x.arc(sx(calibration.sharpness.best_offset_m),sy(Math.min(...p.map(q=>q.score))),5,0,Math.PI*2);x.fill()};function plotMap(id,key){const [c,x]=context(id),p=calibration.maps[key];if(!p.length)return;const loX=Math.min(...p.map(q=>q[0])),hiX=Math.max(...p.map(q=>q[0])),loY=Math.min(...p.map(q=>q[1])),hiY=Math.max(...p.map(q=>q[1]));const sx=v=>20+(v-loX)/(hiX-loX||1)*560,sy=v=>340-(v-loY)/(hiY-loY||1)*320;x.fillStyle='#475569';p.forEach(q=>{x.beginPath();x.arc(sx(q[0]),sy(q[1]),1.5,0,Math.PI*2);x.fill()})};plotCenters();plotSharpness();plotMap('recorded-map','recorded');plotMap('estimated-map','estimated');</script></main></body></html>)HTML";
  return html.str();
}

void atomicWrite(const std::filesystem::path& path, const std::string& content) {
  static std::uint64_t counter = 0;
  const auto temporary = path.string() + ".tmp." + std::to_string(++counter);
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) throw std::runtime_error("cannot create temporary output " + temporary);
    output << content;
    output.flush();
    if (!output) throw std::runtime_error("cannot finish temporary output " + temporary);
  }
  std::error_code error;
  std::filesystem::rename(temporary, path, error);
  if (error) {
    std::filesystem::remove(temporary);
    throw std::runtime_error("cannot atomically publish output " + path.string() + ": " + error.message());
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
  atomicWrite(output / "calibration.json", jsonDocument(run));
  atomicWrite(output / "centers.csv", centersCsv(run));
  atomicWrite(output / "sharpness.csv", sharpnessCsv(run));
  atomicWrite(output / "report.html", reportHtml(run));
}

}  // namespace orb_lidar_mapper
