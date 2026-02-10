# 抖动指标
'''
"csv_dir": LaunchConfiguration("csv_dir"),
"csv_path": LaunchConfiguration("csv_path"),
"output_csv": LaunchConfiguration("output_csv"),
"arm_mode": LaunchConfiguration("arm_mode"),
"curvature_s": LaunchConfiguration("curvature_s"),
"curvature_plot": LaunchConfiguration("curvature_plot"),
"fs": LaunchConfiguration("fs"),
"fc_ratio": LaunchConfiguration("fc_ratio"),
"amp_th": LaunchConfiguration("amp_th"),
"smoothness_plot": LaunchConfiguration("smoothness_plot"),
"w_c": LaunchConfiguration("w_c"),
'''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="D_JEARM", package_name="double_jearm_moveit_config")
        .robot_description(file_path="config/D_JEARM.urdf.xacro")
        .robot_description_semantic(file_path="config/D_JEARM.srdf")
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "csv_dir",
                default_value="/home/wangyunhao/development/ros2_ws/src/double_jearm_moveit_config/csv",
                description="Directory containing CSV files with joint positions.",
            ),
            DeclareLaunchArgument(
                "csv_path",
                default_value="/home/wangyunhao/development/ros2_ws/src/double_jearm_moveit_config/test_joint_state_14.csv",
                description="Path to a single CSV file (used if csv_dir is empty).",
            ),
            DeclareLaunchArgument(
                "output_csv",
                default_value="",
                description="Output CSV path for fused smoothness scores (empty means csv_dir/smoothness_scores.csv).",
            ),
            DeclareLaunchArgument(
                "arm_mode",
                default_value="both",
                description="left/right/both to select arm joints.",
            ),
            DeclareLaunchArgument(
                "curvature_s",
                default_value="0.0",
                description="Resample distance for curvature (0 means no resample).",
            ),
            DeclareLaunchArgument(
                "curvature_plot",
                default_value="false",
                description="Whether to plot curvature (not implemented in C++).",
            ),
            DeclareLaunchArgument(
                "fs",
                default_value="100.0",
                description="Sampling frequency (Hz).",
            ),
            DeclareLaunchArgument(
                "fc_ratio",
                default_value="0.25",
                description="High-frequency threshold ratio of Nyquist.",
            ),
            DeclareLaunchArgument(
                "amp_th",
                default_value="0.0",
                description="Amplitude threshold for strong high-frequency components.",
            ),
            DeclareLaunchArgument(
                "smoothness_plot",
                default_value="false",
                description="Whether to plot spectrum (not implemented in C++).",
            ),
            DeclareLaunchArgument(
                "w_c",
                default_value="0.5",
                description="Curvature weight for fused smoothness score (0~1).",
            ),
            Node(
                package="double_jearm_moveit_config",
                executable="jitter_metrics",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    {
                        "csv_dir": LaunchConfiguration("csv_dir"),
                        "csv_path": LaunchConfiguration("csv_path"),
                        "output_csv": LaunchConfiguration("output_csv"),
                        "arm_mode": LaunchConfiguration("arm_mode"),
                        "curvature_s": LaunchConfiguration("curvature_s"),
                        "curvature_plot": LaunchConfiguration("curvature_plot"),
                        "fs": LaunchConfiguration("fs"),
                        "fc_ratio": LaunchConfiguration("fc_ratio"),
                        "amp_th": LaunchConfiguration("amp_th"),
                        "smoothness_plot": LaunchConfiguration("smoothness_plot"),
                        "w_c": LaunchConfiguration("w_c"),
                    },
                ],
            ),
        ]
    )
