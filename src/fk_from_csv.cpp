#include <rclcpp/rclcpp.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>

#include <cctype>
#include <cstddef>
#include <fstream>
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
    values.reserve(columns);
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

std::string join_strings(const std::vector<std::string> &values)
{
  std::ostringstream oss;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << values[i];
  }
  return oss.str();
}

void log_fk_result(const rclcpp::Logger &logger,
                   size_t row_index,
                   const std::string &group_name,
                   const std::string &ref_link,
                   const std::string &tip_link,
                   const Eigen::Isometry3d &transform)
{
  const Eigen::Vector3d position = transform.translation();
  const Eigen::Quaterniond quat(transform.rotation());
  RCLCPP_INFO(logger,
              "row %zu group %s ref %s tip %s pos [%.6f, %.6f, %.6f] quat [%.6f, %.6f, %.6f, %.6f]",
              row_index,
              group_name.c_str(),
              ref_link.c_str(),
              tip_link.c_str(),
              position.x(),
              position.y(),
              position.z(),
              quat.x(),
              quat.y(),
              quat.z(),
              quat.w());
}
}  // namespace

template <typename T>
T get_or_declare_param(rclcpp::Node &node, const std::string &name, const T &default_value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
  T value = default_value;
  node.get_parameter(name, value);
  return value;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("fk_from_csv", "", node_options);
  auto logger = node->get_logger();

  const std::string group1_name = "jearm_group1";
  const std::string group2_name = "jearm_group2";
  const std::string tip_link1 = "Link17";
  const std::string tip_link2 = "Link27";
  const std::string ref_link1 = "Link10";
  const std::string ref_link2 = "Link20";

  const std::string csv_path = get_or_declare_param<std::string>(*node, "csv_path", "");
  const std::string arm_mode_str = get_or_declare_param<std::string>(*node, "arm_mode", "both");

  ArmMode arm_mode = ArmMode::Both;
  const std::string arm_mode_lower = to_lower_copy(arm_mode_str);
  if (arm_mode_lower == "left") {
    arm_mode = ArmMode::Left;
  } else if (arm_mode_lower == "right") {
    arm_mode = ArmMode::Right;
  } else if (arm_mode_lower == "both") {
    arm_mode = ArmMode::Both;
  } else {
    RCLCPP_ERROR(logger, "Invalid arm_mode: %s (expected left/right/both).", arm_mode_str.c_str());
    rclcpp::shutdown();
    return 1;
  }

  std::vector<std::vector<double>> rows;
  size_t columns = 0;
  try {
    if (!parse_csv_file(csv_path, rows, columns)) {
      RCLCPP_ERROR(logger, "Failed to open CSV file: %s", csv_path.c_str());
      rclcpp::shutdown();
      return 1;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "CSV parse error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Loaded %zu rows from %s.", rows.size(), csv_path.c_str());

  if (rows.empty()) {
    RCLCPP_ERROR(logger, "CSV file has no data rows: %s", csv_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr &robot_model = robot_model_loader.getModel();
  if (!robot_model) {
    RCLCPP_ERROR(logger, "Failed to load robot model. Is robot_description set?");
    rclcpp::shutdown();
    return 1;
  }

  const auto *group1 = robot_model->getJointModelGroup(group1_name);
  const auto *group2 = robot_model->getJointModelGroup(group2_name);
  if (!group1 || !group2) {
    RCLCPP_ERROR(logger, "Joint model group not found. group1=%s group2=%s", group1_name.c_str(), group2_name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  const size_t group1_vars = group1->getVariableCount();
  const size_t group2_vars = group2->getVariableCount();
  RCLCPP_INFO(logger, "Group1 %s variables (%zu): %s", group1_name.c_str(), group1_vars,
              join_strings(group1->getVariableNames()).c_str());
  RCLCPP_INFO(logger, "Group2 %s variables (%zu): %s", group2_name.c_str(), group2_vars,
              join_strings(group2->getVariableNames()).c_str());

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();

  if (arm_mode == ArmMode::Left) {
    if (columns != group1_vars) {
      RCLCPP_ERROR(logger, "arm_mode=left requires %zu columns (got %zu).", group1_vars, columns);
      rclcpp::shutdown();
      return 1;
    }
    if (!robot_model->getLinkModel(tip_link1) || !robot_model->getLinkModel(ref_link1)) {
      RCLCPP_ERROR(logger, "Reference/tip link not found for left arm.");
      rclcpp::shutdown();
      return 1;
    }

    for (size_t i = 0; i < rows.size(); ++i) {
      const auto &row = rows[i];
      if (row.size() != columns) {
        RCLCPP_ERROR(logger, "Row %zu column mismatch.", i);
        rclcpp::shutdown();
        return 1;
      }
      state.setJointGroupPositions(group1, row);
      state.update();
      const auto &t_ref = state.getGlobalLinkTransform(ref_link1);
      const auto &t_tip = state.getGlobalLinkTransform(tip_link1);
      const Eigen::Isometry3d transform = t_ref.inverse() * t_tip;
      log_fk_result(logger, i, group1_name, ref_link1, tip_link1, transform);
    }
  } else if (arm_mode == ArmMode::Right) {
    if (columns != group2_vars) {
      RCLCPP_ERROR(logger, "arm_mode=right requires %zu columns (got %zu).", group2_vars, columns);
      rclcpp::shutdown();
      return 1;
    }
    if (!robot_model->getLinkModel(tip_link2) || !robot_model->getLinkModel(ref_link2)) {
      RCLCPP_ERROR(logger, "Reference/tip link not found for right arm.");
      rclcpp::shutdown();
      return 1;
    }

    for (size_t i = 0; i < rows.size(); ++i) {
      const auto &row = rows[i];
      if (row.size() != columns) {
        RCLCPP_ERROR(logger, "Row %zu column mismatch.", i);
        rclcpp::shutdown();
        return 1;
      }
      state.setJointGroupPositions(group2, row);
      state.update();
      const auto &t_ref = state.getGlobalLinkTransform(ref_link2);
      const auto &t_tip = state.getGlobalLinkTransform(tip_link2);
      const Eigen::Isometry3d transform = t_ref.inverse() * t_tip;
      log_fk_result(logger, i, group2_name, ref_link2, tip_link2, transform);
    }
  } else {
    if (columns != group1_vars + group2_vars) {
      RCLCPP_ERROR(logger, "arm_mode=both requires %zu columns (got %zu).",
                   group1_vars + group2_vars, columns);
      rclcpp::shutdown();
      return 1;
    }
    if (!robot_model->getLinkModel(tip_link1) || !robot_model->getLinkModel(tip_link2)) {
      RCLCPP_ERROR(logger, "Tip link not found: %s or %s", tip_link1.c_str(), tip_link2.c_str());
      rclcpp::shutdown();
      return 1;
    }
    if (!robot_model->getLinkModel(ref_link1) || !robot_model->getLinkModel(ref_link2)) {
      RCLCPP_ERROR(logger, "Reference link not found: %s or %s", ref_link1.c_str(), ref_link2.c_str());
      rclcpp::shutdown();
      return 1;
    }

    for (size_t i = 0; i < rows.size(); ++i) {
      const auto &row = rows[i];
      if (row.size() != columns) {
        RCLCPP_ERROR(logger, "Row %zu column mismatch.", i);
        rclcpp::shutdown();
        return 1;
      }

      const auto split = row.begin() + static_cast<std::ptrdiff_t>(group1_vars);
      std::vector<double> group1_values(row.begin(), split);
      std::vector<double> group2_values(split, row.end());
      state.setJointGroupPositions(group1, group1_values);
      state.setJointGroupPositions(group2, group2_values);
      state.update();

      const auto &t_ref1 = state.getGlobalLinkTransform(ref_link1);
      const auto &t_tip1 = state.getGlobalLinkTransform(tip_link1);
      const Eigen::Isometry3d transform1 = t_ref1.inverse() * t_tip1;

      const auto &t_ref2 = state.getGlobalLinkTransform(ref_link2);
      const auto &t_tip2 = state.getGlobalLinkTransform(tip_link2);
      const Eigen::Isometry3d transform2 = t_ref2.inverse() * t_tip2;

      log_fk_result(logger, i, group1_name, ref_link1, tip_link1, transform1);
      log_fk_result(logger, i, group2_name, ref_link2, tip_link2, transform2);
    }
  }

  rclcpp::shutdown();
  return 0;
}
