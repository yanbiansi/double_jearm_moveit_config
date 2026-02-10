# 轨迹复现，启动前需要启动rviz_robot_model.launch文件
'''
输入：
"csv_path",default_value="/home/wangyunhao/development/ros2_ws/src/double_jearm_moveit_config/test_joint_state_14.csv",
"publish_rate",default_value="1.0",
"loop",default_value="true",
"arm_mode",default_value="both",
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
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "csv_path",
                default_value="/home/wangyunhao/development/ros2_ws/src/double_jearm_moveit_config/test_joint_state_7.csv",
                description="Path to CSV file with joint positions (relative to current working directory).",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="1.0",
                description="Publish rate in Hz.",
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="true",
                description="Loop playback when reaching the end.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description="Optional frame_id for JointState header.",
            ),
            DeclareLaunchArgument(
                "arm_mode",
                default_value="left",
                description="left/right/both to select arm joints.",
            ),
            Node(
                package="double_jearm_moveit_config",
                executable="joint_state_replay",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {
                        "csv_path": LaunchConfiguration("csv_path"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                        "loop": LaunchConfiguration("loop"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "arm_mode": LaunchConfiguration("arm_mode"),
                    }
                ],
            ),
        ]
    )
