#include "orb_slam3_wrapper/stereo_calib_report.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "orb_slam3_wrapper/stereo_calib_analysis.hpp"

namespace orb_slam3_wrapper {
namespace {

std::string jsonEscape(const std::string& value) {
  std::string result;
  for (const char character : value) {
    switch (character) {
      case '\\':
        result += "\\\\";
        break;
      case '"':
        result += "\\\"";
        break;
      case '\n':
        result += "\\n";
        break;
      case '\r':
        result += "\\r";
        break;
      case '\t':
        result += "\\t";
        break;
      case '<':
        result += "\\u003c";
        break;
      default:
        result += character;
        break;
    }
  }
  return result;
}

const char* resultName(ResultClass result) {
  switch (result) {
    case ResultClass::kConsistent:
      return "CONSISTENT";
    case ResultClass::kLikelyOffsetError:
      return "LIKELY_OFFSET_ERROR";
    case ResultClass::kInconclusive:
      return "INCONCLUSIVE";
  }
  return "INCONCLUSIVE";
}

const char* exitCodeMeaning(ResultClass result) {
  switch (result) {
    case ResultClass::kConsistent:
      return "0: CONSISTENT — estimate reliable and agrees with recorded mount";
    case ResultClass::kLikelyOffsetError:
      return "2: LIKELY_OFFSET_ERROR — estimate reliable but disagrees with "
             "recorded mount";
    case ResultClass::kInconclusive:
      return "3: INCONCLUSIVE — weak tracking, few pairs, broad CI, or bad "
             "sector coverage";
  }
  return "3: INCONCLUSIVE";
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
    if (character == '"')
      escaped += "\"\"";
    else
      escaped += character;
  }
  escaped += '"';
  return escaped;
}

void appendMountXy(std::ostringstream& json, const char* key,
                   const MountXy& xy) {
  json << "\"" << key << "\":{\"x_m\":" << number(xy.x_m)
       << ",\"y_m\":" << number(xy.y_m) << "}";
}

void appendIsometry(std::ostringstream& json, const char* key,
                    const Eigen::Isometry3d& T) {
  const Eigen::Vector3d t = T.translation();
  const Eigen::Quaterniond q(T.linear());
  json << "\"" << key << "\":{"
       << "\"x\":" << number(t.x()) << ","
       << "\"y\":" << number(t.y()) << ","
       << "\"z\":" << number(t.z()) << ","
       << "\"qx\":" << number(q.x()) << ","
       << "\"qy\":" << number(q.y()) << ","
       << "\"qz\":" << number(q.z()) << ","
       << "\"qw\":" << number(q.w()) << "}";
}

std::string jsonDocument(const StereoCalibRun& run) {
  std::ostringstream json;
  const auto& est = run.aggregate.estimate;
  const auto& thr = run.config.thresholds;

  json << "{";
  json << "\"result_class\":\"" << resultName(run.aggregate.result_class)
       << "\",";
  json << "\"exit_code\":" << resultExitCode(run.aggregate.result_class) << ",";
  json << "\"exit_code_meaning\":\""
       << jsonEscape(exitCodeMeaning(run.aggregate.result_class)) << "\",";
  json << "\"summary\":\"" << jsonEscape(run.aggregate.summary) << "\",";
  json << "\"bag_path\":\"" << jsonEscape(run.config.bag_path.string())
       << "\",";
  json << "\"output_dir\":\"" << jsonEscape(run.config.output_dir.string())
       << "\",";
  json << "\"overwrite\":" << (run.config.overwrite ? "true" : "false") << ",";

  appendMountXy(json, "recorded_camera_link_xy", run.aggregate.recorded_xy);
  json << ",";
  appendMountXy(json, "estimated_camera_link_xy", est.median_xy);
  json << ",";
  appendMountXy(json, "delta_xy", run.aggregate.delta_xy);
  json << ",";
  appendMountXy(json, "ci_half_width", est.ci_half_width);
  json << ",";

  json << "\"accepted_pairs\":" << est.accepted_pairs << ",";
  json << "\"sectors_used\":" << est.sectors_used << ",";
  json << "\"tracked_ok\":" << run.tracked_ok << ",";
  json << "\"tracked_total\":" << run.tracked_total << ",";
  json << "\"estimate_reliable\":" << (est.reliable ? "true" : "false") << ",";

  json << "\"unreliable_reasons\":[";
  for (std::size_t i = 0; i < est.unreliable_reasons.size(); ++i) {
    if (i != 0) json << ',';
    json << "\"" << jsonEscape(est.unreliable_reasons[i]) << "\"";
  }
  json << "],";

  json << "\"thresholds\":{"
       << "\"min_accepted_pairs\":" << thr.min_accepted_pairs << ","
       << "\"min_sectors\":" << thr.min_sectors << ","
       << "\"max_ci_half_width_m\":" << number(thr.max_ci_half_width_m) << ","
       << "\"agreement_floor_m\":" << number(thr.agreement_floor_m) << ","
       << "\"max_abs_center_m\":" << number(thr.max_abs_center_m) << ","
       << "\"min_pair_yaw_rad\":" << number(thr.min_pair_yaw_rad) << ","
       << "\"max_pair_yaw_rad\":" << number(thr.max_pair_yaw_rad) << ","
       << "\"max_tracking_loss_fraction\":"
       << number(run.config.max_tracking_loss_fraction) << ","
       << "\"min_tracked_frames\":" << run.config.min_tracked_frames << "},";

  appendIsometry(json, "recorded_T_base_camera_link",
                 run.dataset.recorded_mount.T_base_camera_link);
  json << ",";
  appendIsometry(json, "recorded_T_camera_link_left_optical",
                 run.dataset.recorded_mount.T_camera_link_left_optical);
  json << ",";

  json << "\"center_samples\":[";
  for (std::size_t i = 0; i < run.samples.size(); ++i) {
    if (i != 0) json << ',';
    const auto& s = run.samples[i];
    json << "{\"source_index\":" << s.source_index
         << ",\"target_index\":" << s.target_index
         << ",\"yaw_sector\":" << s.yaw_sector
         << ",\"accepted\":" << (s.accepted ? "true" : "false")
         << ",\"center_x_m\":" << number(s.center.x)
         << ",\"center_y_m\":" << number(s.center.y)
         << ",\"mount_x_m\":" << number(s.mount_xy.x_m)
         << ",\"mount_y_m\":" << number(s.mount_xy.y_m)
         << ",\"rejection_reason\":\"" << jsonEscape(s.rejection_reason)
         << "\"}";
  }
  json << "],";

  json << "\"trajectory\":[";
  for (std::size_t i = 0; i < run.trajectory.size(); ++i) {
    if (i != 0) json << ',';
    const auto& tp = run.trajectory[i];
    const Eigen::Vector3d t = tp.T_world_optical.translation();
    const Eigen::Quaterniond q(tp.T_world_optical.linear());
    json << "{\"stamp_ns\":" << tp.stamp_ns
         << ",\"x\":" << number(t.x()) << ",\"y\":" << number(t.y())
         << ",\"z\":" << number(t.z()) << ",\"qx\":" << number(q.x())
         << ",\"qy\":" << number(q.y()) << ",\"qz\":" << number(q.z())
         << ",\"qw\":" << number(q.w())
         << ",\"tracking_state\":" << tp.tracking_state
         << ",\"pose_valid\":" << (tp.pose_valid ? "true" : "false") << "}";
  }
  json << "],";

  json << "\"planar\":[";
  for (std::size_t i = 0; i < run.planar.size(); ++i) {
    if (i != 0) json << ',';
    const auto& p = run.planar[i];
    json << "{\"stamp_ns\":" << p.stamp_ns << ",\"x\":" << number(p.pose.x)
         << ",\"y\":" << number(p.pose.y) << ",\"yaw\":" << number(p.pose.yaw)
         << ",\"height_m\":" << number(p.height_m)
         << ",\"valid\":" << (p.valid ? "true" : "false") << "}";
  }
  json << "]";

  json << "}";
  return json.str();
}

std::string centersCsv(const StereoCalibRun& run) {
  std::ostringstream csv;
  csv << "source_index,target_index,yaw_sector,accepted,center_x_m,center_y_m,"
         "mount_x_m,mount_y_m,rejection_reason\n";
  for (const auto& s : run.samples) {
    csv << s.source_index << ',' << s.target_index << ',' << s.yaw_sector << ','
        << (s.accepted ? "true" : "false") << ',' << number(s.center.x) << ','
        << number(s.center.y) << ',' << number(s.mount_xy.x_m) << ','
        << number(s.mount_xy.y_m) << ',' << csvField(s.rejection_reason)
        << '\n';
  }
  return csv.str();
}

std::string trajectoryCsv(const StereoCalibRun& run) {
  std::ostringstream csv;
  csv << "stamp_ns,x,y,z,qx,qy,qz,qw,tracking_state,pose_valid\n";
  for (const auto& tp : run.trajectory) {
    const Eigen::Vector3d t = tp.T_world_optical.translation();
    const Eigen::Quaterniond q(tp.T_world_optical.linear());
    csv << tp.stamp_ns << ',' << csvNumber(t.x()) << ',' << csvNumber(t.y())
        << ',' << csvNumber(t.z()) << ',' << csvNumber(q.x()) << ','
        << csvNumber(q.y()) << ',' << csvNumber(q.z()) << ','
        << csvNumber(q.w()) << ',' << tp.tracking_state << ','
        << (tp.pose_valid ? "true" : "false") << '\n';
  }
  return csv.str();
}

// Minimal static SVG for top-down planar trajectory + center scatter.
std::string buildSvgPlots(const StereoCalibRun& run) {
  constexpr double W = 420.0;
  constexpr double H = 320.0;
  constexpr double pad = 24.0;

  auto bounds = [](const std::vector<double>& xs, const std::vector<double>& ys,
                   double& min_x, double& max_x, double& min_y, double& max_y) {
    min_x = max_x = 0.0;
    min_y = max_y = 0.0;
    bool any = false;
    for (std::size_t i = 0; i < xs.size(); ++i) {
      if (!std::isfinite(xs[i]) || !std::isfinite(ys[i])) continue;
      if (!any) {
        min_x = max_x = xs[i];
        min_y = max_y = ys[i];
        any = true;
      } else {
        min_x = std::min(min_x, xs[i]);
        max_x = std::max(max_x, xs[i]);
        min_y = std::min(min_y, ys[i]);
        max_y = std::max(max_y, ys[i]);
      }
    }
    if (!any) {
      min_x = -0.5;
      max_x = 0.5;
      min_y = -0.5;
      max_y = 0.5;
    }
    const double dx = std::max(max_x - min_x, 0.05);
    const double dy = std::max(max_y - min_y, 0.05);
    min_x -= 0.1 * dx;
    max_x += 0.1 * dx;
    min_y -= 0.1 * dy;
    max_y += 0.1 * dy;
  };

  auto project = [&](double x, double y, double min_x, double max_x,
                     double min_y, double max_y) {
    const double sx = pad + (x - min_x) / (max_x - min_x) * (W - 2 * pad);
    // SVG y grows down; flip so +y is up.
    const double sy = H - pad - (y - min_y) / (max_y - min_y) * (H - 2 * pad);
    return std::make_pair(sx, sy);
  };

  std::ostringstream svg;

  // --- Trajectory (planar xy) ---
  std::vector<double> tx, ty;
  for (const auto& p : run.planar) {
    if (!p.valid) continue;
    tx.push_back(p.pose.x);
    ty.push_back(p.pose.y);
  }
  // Fallback to optical translation if planar empty.
  if (tx.empty()) {
    for (const auto& tp : run.trajectory) {
      if (!tp.pose_valid) continue;
      tx.push_back(tp.T_world_optical.translation().x());
      ty.push_back(tp.T_world_optical.translation().y());
    }
  }
  double tminx, tmaxx, tminy, tmaxy;
  bounds(tx, ty, tminx, tmaxx, tminy, tmaxy);

  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << W
      << "\" height=\"" << H
      << "\" viewBox=\"0 0 " << W << " " << H
      << "\" aria-label=\"Top-down trajectory\">"
      << "<rect width=\"100%\" height=\"100%\" fill=\"#fff\" stroke=\"#d9e0e8\"/>"
      << "<text x=\"12\" y=\"18\" font-size=\"12\" fill=\"#334155\">"
         "Top-down trajectory (planar left)</text>";
  // axes cross at origin if in range
  if (tminx <= 0 && tmaxx >= 0) {
    const auto o = project(0, 0, tminx, tmaxx, tminy, tmaxy);
    svg << "<line x1=\"" << o.first << "\" y1=\"" << pad << "\" x2=\""
        << o.first << "\" y2=\"" << (H - pad)
        << "\" stroke=\"#cbd5e1\" stroke-dasharray=\"3 3\"/>";
  }
  if (tminy <= 0 && tmaxy >= 0) {
    const auto o = project(0, 0, tminx, tmaxx, tminy, tmaxy);
    svg << "<line x1=\"" << pad << "\" y1=\"" << o.second << "\" x2=\""
        << (W - pad) << "\" y2=\"" << o.second
        << "\" stroke=\"#cbd5e1\" stroke-dasharray=\"3 3\"/>";
  }
  if (tx.size() >= 2) {
    svg << "<polyline fill=\"none\" stroke=\"#2563eb\" stroke-width=\"1.5\" "
           "points=\"";
    for (std::size_t i = 0; i < tx.size(); ++i) {
      const auto p = project(tx[i], ty[i], tminx, tmaxx, tminy, tmaxy);
      if (i) svg << ' ';
      svg << p.first << ',' << p.second;
    }
    svg << "\"/>";
  }
  for (std::size_t i = 0; i < tx.size(); ++i) {
    const auto p = project(tx[i], ty[i], tminx, tmaxx, tminy, tmaxy);
    svg << "<circle cx=\"" << p.first << "\" cy=\"" << p.second
        << "\" r=\"2.5\" fill=\"#2563eb\"/>";
  }
  // recorded / estimated mount markers near origin diagram is separate;
  // mark base origin
  {
    const auto o = project(0, 0, tminx, tmaxx, tminy, tmaxy);
    svg << "<circle cx=\"" << o.first << "\" cy=\"" << o.second
        << "\" r=\"5\" fill=\"#d4a017\" stroke=\"#92400e\"/>";
  }
  svg << "</svg>";

  // --- Center scatter (mount_xy samples) ---
  std::vector<double> cx, cy;
  std::vector<bool> acc;
  for (const auto& s : run.samples) {
    cx.push_back(s.mount_xy.x_m);
    cy.push_back(s.mount_xy.y_m);
    acc.push_back(s.accepted);
  }
  // Include recorded / estimated for bounds
  cx.push_back(run.aggregate.recorded_xy.x_m);
  cy.push_back(run.aggregate.recorded_xy.y_m);
  acc.push_back(true);
  cx.push_back(run.aggregate.estimate.median_xy.x_m);
  cy.push_back(run.aggregate.estimate.median_xy.y_m);
  acc.push_back(true);

  double cminx, cmaxx, cminy, cmaxy;
  bounds(cx, cy, cminx, cmaxx, cminy, cmaxy);

  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << W
      << "\" height=\"" << H
      << "\" viewBox=\"0 0 " << W << " " << H
      << "\" aria-label=\"Center scatter\">"
      << "<rect width=\"100%\" height=\"100%\" fill=\"#fff\" stroke=\"#d9e0e8\"/>"
      << "<text x=\"12\" y=\"18\" font-size=\"12\" fill=\"#334155\">"
         "Mount xy scatter (accepted filled, rejected outlined)</text>";

  for (std::size_t i = 0; i < run.samples.size(); ++i) {
    if (!std::isfinite(run.samples[i].mount_xy.x_m) ||
        !std::isfinite(run.samples[i].mount_xy.y_m))
      continue;
    const auto p = project(run.samples[i].mount_xy.x_m,
                           run.samples[i].mount_xy.y_m, cminx, cmaxx, cminy,
                           cmaxy);
    if (run.samples[i].accepted) {
      svg << "<circle cx=\"" << p.first << "\" cy=\"" << p.second
          << "\" r=\"4\" fill=\"#16a34a\"/>";
    } else {
      svg << "<circle cx=\"" << p.first << "\" cy=\"" << p.second
          << "\" r=\"5\" fill=\"none\" stroke=\"#dc2626\" stroke-width=\"2\"/>";
    }
  }
  // Recorded (gold) and estimated (blue)
  if (std::isfinite(run.aggregate.recorded_xy.x_m) &&
      std::isfinite(run.aggregate.recorded_xy.y_m)) {
    const auto p =
        project(run.aggregate.recorded_xy.x_m, run.aggregate.recorded_xy.y_m,
                cminx, cmaxx, cminy, cmaxy);
    svg << "<circle cx=\"" << p.first << "\" cy=\"" << p.second
        << "\" r=\"7\" fill=\"#d4a017\"/>";
  }
  if (std::isfinite(run.aggregate.estimate.median_xy.x_m) &&
      std::isfinite(run.aggregate.estimate.median_xy.y_m)) {
    const auto p = project(run.aggregate.estimate.median_xy.x_m,
                           run.aggregate.estimate.median_xy.y_m, cminx, cmaxx,
                           cminy, cmaxy);
    svg << "<circle cx=\"" << p.first << "\" cy=\"" << p.second
        << "\" r=\"6\" fill=\"#2563eb\"/>";
  }
  svg << "</svg>";

  return svg.str();
}

std::string reportHtml(const StereoCalibRun& run) {
  const auto& est = run.aggregate.estimate;
  std::ostringstream html;
  html << R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Stereo rotation-center calibration</title>
<style>
:root{font-family:system-ui,sans-serif;color:#17202a;background:#f5f7fa}
body{margin:0;padding:20px}
main{max-width:1000px;margin:auto}
section{background:#fff;border:1px solid #d9e0e8;border-radius:12px;padding:16px;margin:12px 0;box-shadow:0 1px 3px #10203018}
h1{margin-top:0}
.status{font-size:1.3rem;font-weight:700}
.meta{color:#475569;font-size:.9rem}
table{border-collapse:collapse;width:100%;font-size:.9rem}
th,td{padding:7px;border-bottom:1px solid #e5e9ef;text-align:left}
.plots{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.plots svg{max-width:100%;height:auto;display:block}
.legend{font-size:.85rem;color:#475569}
.legend span{margin-right:12px}
.swatch{display:inline-block;width:.8em;height:.8em;border-radius:50%;margin-right:4px;vertical-align:middle}
@media(max-width:700px){.plots{grid-template-columns:1fr}}
</style></head><body><main>
<h1>Stereo rotation-center calibration</h1>
<section>
<div class="status">)HTML"
       << resultName(run.aggregate.result_class) << R"HTML(</div>
<p class="meta">)HTML"
       << jsonEscape(exitCodeMeaning(run.aggregate.result_class))
       << R"HTML(</p>
<p>)HTML" << jsonEscape(run.aggregate.summary) << R"HTML(</p>
<p class="meta">Bag: )HTML"
       << jsonEscape(run.config.bag_path.string()) << R"HTML(</p>
</section>
<section>
<h2>Comparison</h2>
<table>
<thead><tr><th>Quantity</th><th>x (m)</th><th>y (m)</th></tr></thead>
<tbody>
<tr><td>Recorded camera_link xy</td><td>)HTML"
       << number(run.aggregate.recorded_xy.x_m) << "</td><td>"
       << number(run.aggregate.recorded_xy.y_m) << R"HTML(</td></tr>
<tr><td>Estimated camera_link xy</td><td>)HTML"
       << number(est.median_xy.x_m) << "</td><td>"
       << number(est.median_xy.y_m) << R"HTML(</td></tr>
<tr><td>Delta (est − recorded)</td><td>)HTML"
       << number(run.aggregate.delta_xy.x_m) << "</td><td>"
       << number(run.aggregate.delta_xy.y_m) << R"HTML(</td></tr>
<tr><td>CI half-width (95%)</td><td>)HTML"
       << number(est.ci_half_width.x_m) << "</td><td>"
       << number(est.ci_half_width.y_m) << R"HTML(</td></tr>
</tbody></table>
</section>
<section>
<h2>Tracking &amp; pairs</h2>
<table>
<tbody>
<tr><td>Accepted pairs</td><td>)HTML"
       << est.accepted_pairs << R"HTML(</td></tr>
<tr><td>Sectors used</td><td>)HTML"
       << est.sectors_used << R"HTML(</td></tr>
<tr><td>Tracked OK / total</td><td>)HTML"
       << run.tracked_ok << " / " << run.tracked_total << R"HTML(</td></tr>
<tr><td>Estimate reliable</td><td>)HTML"
       << (est.reliable ? "true" : "false") << R"HTML(</td></tr>
</tbody></table>
)HTML";

  if (!est.unreliable_reasons.empty()) {
    html << "<p><strong>Unreliable reasons:</strong> ";
    for (std::size_t i = 0; i < est.unreliable_reasons.size(); ++i) {
      if (i) html << "; ";
      html << jsonEscape(est.unreliable_reasons[i]);
    }
    html << "</p>";
  }

  html << R"HTML(</section>
<section>
<h2>Thresholds</h2>
<table>
<tbody>
<tr><td>min_accepted_pairs</td><td>)HTML"
       << run.config.thresholds.min_accepted_pairs << R"HTML(</td></tr>
<tr><td>min_sectors</td><td>)HTML"
       << run.config.thresholds.min_sectors << R"HTML(</td></tr>
<tr><td>max_ci_half_width_m</td><td>)HTML"
       << number(run.config.thresholds.max_ci_half_width_m) << R"HTML(</td></tr>
<tr><td>agreement_floor_m</td><td>)HTML"
       << number(run.config.thresholds.agreement_floor_m) << R"HTML(</td></tr>
<tr><td>max_abs_center_m</td><td>)HTML"
       << number(run.config.thresholds.max_abs_center_m) << R"HTML(</td></tr>
</tbody></table>
</section>
<section>
<h2>Plots</h2>
<p class="legend">
<span><i class="swatch" style="background:#2563eb"></i>Trajectory / estimated</span>
<span><i class="swatch" style="background:#16a34a"></i>Accepted sample</span>
<span><i class="swatch" style="background:#dc2626"></i>Rejected sample</span>
<span><i class="swatch" style="background:#d4a017"></i>Recorded / origin</span>
</p>
<div class="plots">
)HTML";
  html << buildSvgPlots(run);
  html << R"HTML(
</div>
</section>
</main></body></html>
)HTML";
  return html.str();
}

void writeFile(const std::filesystem::path& path, const std::string& content) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot open output " + path.string());
  }
  output.write(content.data(), static_cast<std::streamsize>(content.size()));
  output.flush();
  if (!output) {
    throw std::runtime_error("cannot finish output " + path.string());
  }
}

}  // namespace

void writeStereoCalibrationReport(const StereoCalibRun& run) {
  const auto& output = run.config.output_dir;
  std::error_code error;
  if (std::filesystem::exists(output, error)) {
    if (!std::filesystem::is_directory(output, error)) {
      throw std::runtime_error("output path is not a directory");
    }
    if (!run.config.overwrite) {
      throw std::runtime_error("output directory exists; use --overwrite");
    }
  } else {
    if (!std::filesystem::create_directories(output, error) || error) {
      throw std::runtime_error("cannot create output directory: " +
                               error.message());
    }
  }

  // Refuse per-file overwrite when files already exist unless overwrite set
  // (directory-level gate above covers the common case; re-check files for
  // partial dirs created empty).
  const std::vector<std::filesystem::path> files = {
      output / "calibration.json", output / "centers.csv",
      output / "trajectory.csv", output / "report.html"};
  if (!run.config.overwrite) {
    for (const auto& f : files) {
      if (std::filesystem::exists(f, error)) {
        throw std::runtime_error("output file exists; use --overwrite: " +
                                 f.string());
      }
    }
  }

  writeFile(files[0], jsonDocument(run));
  writeFile(files[1], centersCsv(run));
  writeFile(files[2], trajectoryCsv(run));
  writeFile(files[3], reportHtml(run));
}

}  // namespace orb_slam3_wrapper
