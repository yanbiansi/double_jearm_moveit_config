#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>

#include <cctype>
#include <fstream>
#include <memory>
#include <sstream>
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

std::vector<std::string> joint_names_for_columns(size_t columns, ArmMode mode)
{
  if (columns == kShortColumns && mode == ArmMode::Left) {
    return {
      "joint11",
      "joint12",
      "joint13",
      "joint14",
      "joint15",
      "joint16",
      "joint17",
    };
  }
  if (columns == kShortColumns && mode == ArmMode::Right) {
    return {
      "joint21",
      "joint22",
      "joint23",
      "joint24",
      "joint25",
      "joint26",
      "joint27",
    };
  }
  if (columns == kLongColumns && mode == ArmMode::Both) {
    return {
      "joint11",
      "joint12",
      "joint13",
      "joint14",
      "joint15",
      "joint16",
      "joint17",
      "joint21",
      "joint22",
      "joint23",
      "joint24",
      "joint25",
      "joint26",
      "joint27",
    };
  }
  return {};
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

visualization_msgs::msg::Marker make_line_strip_marker(const std::string &ns, int id, float r, float g, float b)
{
  visualization_msgs::msg::Marker marker;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  return marker;
}
}  // namespace

class JointStateReplay : public rclcpp::Node
{
public:
  JointStateReplay()
  : rclcpp::Node(
      "joint_state_replay",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    expected_columns_(0),
    row_index_(0),
    marker_enabled_(false),
    marker_group2_enabled_(false),
    arm_mode_(ArmMode::Both)
  {
    csv_path_ = get_or_declare_param<std::string>("csv_path", "");
    publish_rate_ = get_or_declare_param<double>("publish_rate", 50.0);
    loop_ = get_or_declare_param<bool>("loop", false);
    frame_id_ = get_or_declare_param<std::string>("frame_id", "");
    arm_mode_str_ = get_or_declare_param<std::string>("arm_mode", "both");

    if (csv_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'csv_path' is required.");
      rclcpp::shutdown();// 防止后续spin()阻塞
      return;
    }

    const std::string arm_mode_lower = to_lower_copy(arm_mode_str_);
    if (arm_mode_lower == "left") {
      arm_mode_ = ArmMode::Left;
    } else if (arm_mode_lower == "right") {
      arm_mode_ = ArmMode::Right;
    } else if (arm_mode_lower == "both") {
      arm_mode_ = ArmMode::Both;
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid arm_mode: %s (expected left/right/both).", arm_mode_str_.c_str());
      rclcpp::shutdown();
      return;
    }

    if (!load_csv(csv_path_)) {
      rclcpp::shutdown();
      return;
    }

    joint_names_ = joint_names_for_columns(expected_columns_, arm_mode_);
    if (joint_names_.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to resolve joint names for arm_mode=%s.", arm_mode_str_.c_str());
      rclcpp::shutdown();
      return;
    }

    publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&JointStateReplay::on_timer, this));
  }

  void initialize_fk_marker()
  {
    if (expected_columns_ == 0 || positions_.empty()) {
      return;
    }

    if (arm_mode_ == ArmMode::Left) {
      ref_link1_ = "Link10";
      tip_link1_ = "Link17";
      marker_group2_enabled_ = false;
    } else if (arm_mode_ == ArmMode::Right) {
      ref_link1_ = "Link20";
      tip_link1_ = "Link27";
      marker_group2_enabled_ = false;
    } else {
      ref_link1_ = "Link10";
      tip_link1_ = "Link17";
      ref_link2_ = "Link20";
      tip_link2_ = "Link27";
      marker_group2_enabled_ = true;
    }

    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this());
    robot_model_ = robot_model_loader.getModel();
    if (!robot_model_) {
      RCLCPP_WARN(get_logger(), "Robot model not available. FK marker disabled.");
      return;
    }

    if (!robot_model_->getLinkModel(tip_link1_) || !robot_model_->getLinkModel(ref_link1_)) {
      RCLCPP_WARN(get_logger(), "Reference/tip link not found. FK marker disabled.");
      return;
    }

    if (marker_group2_enabled_) {
      if (!robot_model_->getLinkModel(tip_link2_) || !robot_model_->getLinkModel(ref_link2_)) {
        RCLCPP_WARN(get_logger(), "Reference/tip link for group2 not found. Group2 marker disabled.");
        marker_group2_enabled_ = false;
      }
    }

    robot_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("fk_trajectory", 10);
    marker_group1_ = make_line_strip_marker("group1", 0, 1.0f, 0.2f, 0.2f);
    marker_group2_ = make_line_strip_marker("group2", 1, 0.2f, 0.6f, 1.0f);
    marker_enabled_ = true;
  }

private:
  template <typename T>
  T get_or_declare_param(const std::string &name, const T &default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    T value = default_value;
    get_parameter(name, value);
    return value;
  }

  void publish_markers(const std::vector<double> &row)
  {
    if (!marker_enabled_) {
      return;
    }

    if (row.size() != joint_names_.size()) {
      RCLCPP_WARN(get_logger(), "Marker skipped: joint_names size mismatch.");
      return;
    }

    robot_state_->setVariablePositions(joint_names_, row);
    robot_state_->update();

    const auto &t_ref1 = robot_state_->getGlobalLinkTransform(ref_link1_);
    const auto &t_tip1 = robot_state_->getGlobalLinkTransform(tip_link1_);
    const Eigen::Isometry3d t_ref_tip1 = t_ref1.inverse() * t_tip1;
    geometry_msgs::msg::Point p1;
    p1.x = t_ref_tip1.translation().x();
    p1.y = t_ref_tip1.translation().y();
    p1.z = t_ref_tip1.translation().z();
    marker_group1_.header.stamp = now();
    marker_group1_.header.frame_id = ref_link1_;
    marker_group1_.points.push_back(p1);
    marker_pub_->publish(marker_group1_);

    if (marker_group2_enabled_) {
      const auto &t_ref2 = robot_state_->getGlobalLinkTransform(ref_link2_);
      const auto &t_tip2 = robot_state_->getGlobalLinkTransform(tip_link2_);
      const Eigen::Isometry3d t_ref_tip2 = t_ref2.inverse() * t_tip2;
      geometry_msgs::msg::Point p2;
      p2.x = t_ref_tip2.translation().x();
      p2.y = t_ref_tip2.translation().y();
      p2.z = t_ref_tip2.translation().z();
      marker_group2_.header.stamp = now();
      marker_group2_.header.frame_id = ref_link2_;
      marker_group2_.points.push_back(p2);
      marker_pub_->publish(marker_group2_);
    }
  }

  bool load_csv(const std::string &path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open csv file: %s", path.c_str());
      return false;
    }

    std::string line;
    size_t line_no = 0;

    while (std::getline(file, line)) {    // 第一次调用读第 1 行，第二次读第 2 行
      ++line_no;
      if (line.empty()) {
        continue;
      }

      auto tokens = split_csv_simple(line);
      if (expected_columns_ == 0) {
        expected_columns_ = tokens.size();  // 确定列数
      }

      if (expected_columns_ != kShortColumns && expected_columns_ != kLongColumns) {
        RCLCPP_ERROR(
          get_logger(),
          "Line %zu: unsupported column count %zu (expected %zu or %zu).",
          line_no, expected_columns_, kShortColumns, kLongColumns);
        return false;
      }
      if (arm_mode_ == ArmMode::Both && expected_columns_ != kLongColumns) {
        RCLCPP_ERROR(
          get_logger(),
          "Line %zu: arm_mode=both requires %zu columns (got %zu).",
          line_no, kLongColumns, expected_columns_);
        return false;
      }
      if ((arm_mode_ == ArmMode::Left || arm_mode_ == ArmMode::Right) && expected_columns_ != kShortColumns) {
        RCLCPP_ERROR(
          get_logger(),
          "Line %zu: arm_mode=%s requires %zu columns (got %zu).",
          line_no, arm_mode_str_.c_str(), kShortColumns, expected_columns_);
        return false;
      }

      std::vector<double> values;
      values.reserve(expected_columns_);
      for (const auto &token : tokens) {
        values.push_back(std::stod(token));
      }

      positions_.push_back(values);
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu rows from %s.",
      positions_.size(), path.c_str());
    return true;
  }

  void on_timer()
  {
    if (positions_.empty()) {
      return;
    }

    if (row_index_ >= positions_.size()) {
      if (loop_) {
        row_index_ = 0;
        if (marker_enabled_) {
          marker_group1_.points.clear();
          marker_group2_.points.clear();
        }
      } else {
        RCLCPP_INFO(get_logger(), "Replay finished. Stopping timer.");
        timer_->cancel();
        return;
      }
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    if (!frame_id_.empty()) {
      msg.header.frame_id = frame_id_;
    }
    msg.name = joint_names_;
    msg.position = positions_[row_index_];

    publisher_->publish(msg);
    publish_markers(positions_[row_index_]);
    ++row_index_;
  }

  std::string csv_path_;
  std::vector<std::string> joint_names_;
  double publish_rate_;
  bool loop_;
  std::string frame_id_;
  std::string arm_mode_str_;
  ArmMode arm_mode_;

  std::vector<std::vector<double>> positions_;
  size_t expected_columns_;   // 列数
  size_t row_index_;    // 行数

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  moveit::core::RobotModelPtr robot_model_;
  std::unique_ptr<moveit::core::RobotState> robot_state_;
  visualization_msgs::msg::Marker marker_group1_;
  visualization_msgs::msg::Marker marker_group2_;
  std::string tip_link1_;
  std::string tip_link2_;
  std::string ref_link1_;
  std::string ref_link2_;
  bool marker_enabled_;
  bool marker_group2_enabled_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateReplay>();
  node->initialize_fk_marker();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
