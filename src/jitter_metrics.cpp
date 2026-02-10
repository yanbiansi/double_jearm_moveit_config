#include "smoothness_metrics_utils.h"

#include <rclcpp/rclcpp.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr size_t kShortColumns = 7;
constexpr size_t kLongColumns = 14;

enum class ArmMode
{
  Left,
  Right,
  Both
};

std::vector<std::string> split_csv_simple(const std::string &line)
{
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    tokens.push_back(item);
  }
  return tokens;
}

bool parse_csv_file(const std::string &path, std::vector<std::vector<double>> &rows, size_t &columns)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  size_t line_no = 0;
  while (std::getline(file, line)) {
    ++line_no;
    if (line.empty()) {
      continue;
    }
    const auto tokens = split_csv_simple(line);
    if (columns == 0) {
      columns = tokens.size();
    }
    if (columns != kShortColumns && columns != kLongColumns) {
      throw std::runtime_error(
        "Line " + std::to_string(line_no) + ": unsupported column count " + std::to_string(columns) +
        " (expected " + std::to_string(kShortColumns) + " or " + std::to_string(kLongColumns) + ").");
    }
    std::vector<double> values;
    values.reserve(tokens.size());
    for (const auto &token : tokens) {
      values.push_back(std::stod(token));
    }
    rows.push_back(values);
  }
  return true;
}

std::string to_lower_copy(const std::string &input)
{
  std::string output;
  output.reserve(input.size());
  for (const char c : input) {
    output.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return output;
}

ArmMode parse_arm_mode(const std::string &arm_mode)
{
  const std::string lower = to_lower_copy(arm_mode);
  if (lower == "left") {
    return ArmMode::Left;
  }
  if (lower == "right") {
    return ArmMode::Right;
  }
  if (lower == "both") {
    return ArmMode::Both;
  }
  throw std::runtime_error("arm_mode must be left/right/both");
}

Eigen::Vector3d compute_relative_position(const moveit::core::RobotState &state,
                                          const std::string &ref_link,
                                          const std::string &tip_link)
{
  const auto &t_ref = state.getGlobalLinkTransform(ref_link);
  const auto &t_tip = state.getGlobalLinkTransform(tip_link);
  const Eigen::Isometry3d t_ref_tip = t_ref.inverse() * t_tip;
  return t_ref_tip.translation();
}

std::vector<double> compute_accel_norms(const std::vector<Eigen::Vector3d> &points)
{
  if (points.size() < 2) {
    return {};
  }
  std::vector<double> accel;
  accel.reserve(points.size() - 1);
  for (size_t i = 1; i < points.size(); ++i) {
    accel.push_back((points[i] - points[i - 1]).norm());
  }
  return accel;
}

std::vector<std::string> list_csv_files(const std::string &dir_path)
{
  namespace fs = std::filesystem;
  std::vector<std::string> files;
  const fs::path dir(dir_path);
  if (!fs::exists(dir) || !fs::is_directory(dir)) {
    return files;
  }
  for (const auto &entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const std::string ext = to_lower_copy(entry.path().extension().string());
    if (ext == ".csv") {
      files.push_back(entry.path().string());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

bool write_scores_csv(const std::string &path,
                      const std::vector<std::string> &labels,
                      const std::vector<double> &curv,
                      const std::vector<double> &accel,
                      const SmoothnessScoreResult &scores)
{
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    return false;
  }
  ofs << "label,curvature_hf_strong_energy_n,accel_hf_strong_energy_n,norm_C,norm_A,final_score\n";
  ofs << std::setprecision(12);
  for (size_t i = 0; i < labels.size(); ++i) {
    ofs << labels[i] << ","
        << curv[i] << ","
        << accel[i] << ","
        << scores.norm_C[i] << ","
        << scores.norm_A[i] << ","
        << scores.final_scores[i] << "\n";
  }
  return true;
}
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("jitter_metrics", "", node_options);
  auto logger = node->get_logger();

  std::string csv_dir;
  std::string csv_path;
  std::string output_csv;
  std::string arm_mode;
  double curvature_s = 0.0;
  bool curvature_plot = false;
  double fs = 0.0;
  double fc_ratio = 0.0;
  double amp_th = 0.0;
  bool smoothness_plot = false;
  double w_c = 0.5;

  node->get_parameter("csv_dir", csv_dir);
  node->get_parameter("csv_path", csv_path);
  node->get_parameter("output_csv", output_csv);
  node->get_parameter("arm_mode", arm_mode);
  node->get_parameter("curvature_s", curvature_s);
  node->get_parameter("curvature_plot", curvature_plot);
  node->get_parameter("fs", fs);
  node->get_parameter("fc_ratio", fc_ratio);
  node->get_parameter("amp_th", amp_th);
  node->get_parameter("smoothness_plot", smoothness_plot);
  node->get_parameter("w_c", w_c);

  std::vector<std::string> csv_files;
  if (!csv_dir.empty()) {
    csv_files = list_csv_files(csv_dir);
    if (csv_files.empty()) {
      RCLCPP_ERROR(logger, "No csv files found in directory: %s", csv_dir.c_str());
      rclcpp::shutdown();
      return 1;
    }
  } else if (!csv_path.empty()) {
    csv_files.push_back(csv_path);
  } else {
    RCLCPP_ERROR(logger, "Parameter 'csv_dir' (or 'csv_path') is required.");
    rclcpp::shutdown();
    return 1;
  }

  ArmMode mode = ArmMode::Both;
  try {
    mode = parse_arm_mode(arm_mode);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Invalid arm_mode: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr &robot_model = robot_model_loader.getModel();
  if (!robot_model) {
    RCLCPP_ERROR(logger, "Robot model not available (robot_description missing).");
    rclcpp::shutdown();
    return 1;
  }

  const std::string group1_name = "jearm_group1";
  const std::string group2_name = "jearm_group2";
  const std::string ref_link1 = "Link10";
  const std::string tip_link1 = "Link17";
  const std::string ref_link2 = "Link20";
  const std::string tip_link2 = "Link27";

  const auto *group1 = robot_model->getJointModelGroup(group1_name);
  const auto *group2 = robot_model->getJointModelGroup(group2_name);
  if (!group1 || !group2) {
    RCLCPP_ERROR(logger, "Joint model group not found.");
    rclcpp::shutdown();
    return 1;
  }

  const size_t group1_vars = group1->getVariableCount();
  const size_t group2_vars = group2->getVariableCount();

  if (mode == ArmMode::Left || mode == ArmMode::Both) {
    if (!robot_model->getLinkModel(ref_link1) || !robot_model->getLinkModel(tip_link1)) {
      RCLCPP_ERROR(logger, "Left ref/tip link not found.");
      rclcpp::shutdown();
      return 1;
    }
  }
  if (mode == ArmMode::Right || mode == ArmMode::Both) {
    if (!robot_model->getLinkModel(ref_link2) || !robot_model->getLinkModel(tip_link2)) {
      RCLCPP_ERROR(logger, "Right ref/tip link not found.");
      rclcpp::shutdown();
      return 1;
    }
  }

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();

  auto print_metrics = [&](const std::string &label,
                           const CurvatureResult &curv,
                           const SmoothnessFrequencyMetrics &sm_curv,
                           const SmoothnessFrequencyMetrics &sm_accel) {
    RCLCPP_INFO(logger, "[%s] curvature points=%zu", label.c_str(), curv.kappa.size());
    RCLCPP_INFO(logger,
                "[%s] curvature_hf_strong_energy_n=%.6e accel_hf_strong_energy_n=%.6e",
                label.c_str(),
                sm_curv.hf_strong_energy_n,
                sm_accel.hf_strong_energy_n);
  };

  bool all_ok = true;
  std::vector<std::string> score_labels;
  std::vector<double> score_curv;
  std::vector<double> score_accel;
  for (const auto &csv_file : csv_files) {
    RCLCPP_INFO(logger, "Processing csv: %s", csv_file.c_str());

    std::vector<std::vector<double>> rows;
    size_t columns = 0;
    try {
      if (!parse_csv_file(csv_file, rows, columns)) {
        RCLCPP_ERROR(logger, "Failed to open csv: %s", csv_file.c_str());
        all_ok = false;
        continue;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger, "CSV parse error (%s): %s", csv_file.c_str(), e.what());
      all_ok = false;
      continue;
    }
    if (rows.size() < 3) {
      RCLCPP_ERROR(logger, "CSV has fewer than 3 rows: %s", csv_file.c_str());
      all_ok = false;
      continue;
    }

    if (mode == ArmMode::Left && columns != group1_vars) {
      RCLCPP_ERROR(logger, "arm_mode=left requires %zu columns (got %zu) in %s.",
                   group1_vars, columns, csv_file.c_str());
      all_ok = false;
      continue;
    }
    if (mode == ArmMode::Right && columns != group2_vars) {
      RCLCPP_ERROR(logger, "arm_mode=right requires %zu columns (got %zu) in %s.",
                   group2_vars, columns, csv_file.c_str());
      all_ok = false;
      continue;
    }
    if (mode == ArmMode::Both && columns != group1_vars + group2_vars) {
      RCLCPP_ERROR(logger, "arm_mode=both requires %zu columns (got %zu) in %s.",
                   group1_vars + group2_vars, columns, csv_file.c_str());
      all_ok = false;
      continue;
    }

    std::vector<Eigen::Vector3d> points_left;
    std::vector<Eigen::Vector3d> points_right;
    points_left.reserve(rows.size());
    points_right.reserve(rows.size());

    for (const auto &row : rows) {
      if (row.size() != columns) {
        RCLCPP_ERROR(logger, "Row column mismatch in csv: %s", csv_file.c_str());
        all_ok = false;
        break;
      }

      if (mode == ArmMode::Left) {
        state.setJointGroupPositions(group1, row);
        state.update();
        points_left.push_back(compute_relative_position(state, ref_link1, tip_link1));
      } else if (mode == ArmMode::Right) {
        state.setJointGroupPositions(group2, row);
        state.update();
        points_right.push_back(compute_relative_position(state, ref_link2, tip_link2));
      } else {
        const auto split = row.begin() + static_cast<std::ptrdiff_t>(group1_vars);
        std::vector<double> left_vals(row.begin(), split);
        std::vector<double> right_vals(split, row.end());
        state.setJointGroupPositions(group1, left_vals);
        state.setJointGroupPositions(group2, right_vals);
        state.update();
        points_left.push_back(compute_relative_position(state, ref_link1, tip_link1));
        points_right.push_back(compute_relative_position(state, ref_link2, tip_link2));
      }
    }

    if (mode == ArmMode::Left || mode == ArmMode::Both) {
      CurvatureResult curvature_left = compute_discrete_curvature_3d(points_left, curvature_s, curvature_plot);
      SmoothnessFrequencyMetrics smoothness_left = compute_smoothness_frequency_metrics(
        curvature_left.kappa, fs, fc_ratio, amp_th, smoothness_plot);
      const std::vector<double> accel_left = compute_accel_norms(points_left);
      const SmoothnessFrequencyMetrics accel_smooth_left = compute_smoothness_frequency_metrics(
        accel_left, fs, fc_ratio, amp_th, smoothness_plot);
      const std::string label = "left " + csv_file;
      print_metrics(label, curvature_left, smoothness_left, accel_smooth_left);
      score_labels.push_back(label);
      score_curv.push_back(smoothness_left.hf_strong_energy_n);
      score_accel.push_back(accel_smooth_left.hf_strong_energy_n);
    }
    if (mode == ArmMode::Right || mode == ArmMode::Both) {
      CurvatureResult curvature_right = compute_discrete_curvature_3d(points_right, curvature_s, curvature_plot);
      SmoothnessFrequencyMetrics smoothness_right = compute_smoothness_frequency_metrics(
        curvature_right.kappa, fs, fc_ratio, amp_th, smoothness_plot);
      const std::vector<double> accel_right = compute_accel_norms(points_right);
      const SmoothnessFrequencyMetrics accel_smooth_right = compute_smoothness_frequency_metrics(
        accel_right, fs, fc_ratio, amp_th, smoothness_plot);
      const std::string label = "right " + csv_file;
      print_metrics(label, curvature_right, smoothness_right, accel_smooth_right);
      score_labels.push_back(label);
      score_curv.push_back(smoothness_right.hf_strong_energy_n);
      score_accel.push_back(accel_smooth_right.hf_strong_energy_n);
    }
  }

  if (!score_labels.empty() && !csv_dir.empty()) {
    if (output_csv.empty()) {
      output_csv = (std::filesystem::path(csv_dir) / "smoothness_scores.csv").string();
    }
    try {
      const SmoothnessScoreResult scores = calc_smoothness_score(score_curv, score_accel, w_c);
      if (!write_scores_csv(output_csv, score_labels, score_curv, score_accel, scores)) {
        RCLCPP_ERROR(logger, "Failed to write output csv: %s", output_csv.c_str());
        all_ok = false;
      } else {
        RCLCPP_INFO(logger, "Wrote smoothness scores to %s", output_csv.c_str());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger, "Smoothness score fusion error: %s", e.what());
      all_ok = false;
    }
  }

  rclcpp::shutdown();
  return all_ok ? 0 : 1;
}
