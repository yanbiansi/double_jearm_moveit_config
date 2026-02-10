# 计算前向运动学
'''
输入：
"csv_path",default_value="/home/wangyunhao/development/ros2_ws/src/double_jearm_moveit_config/test_joint_state_14.csv",
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
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "csv_path",
                default_value="/home/wangyunhao/development/ros2_ws/src/double_jearm_moveit_config/test_joint_state_14.csv",
                description="Path to CSV file with joint positions.",
            ),
            DeclareLaunchArgument(
                "arm_mode",
                default_value="both",
                description="left/right/both to select arm joints.",
            ),
            Node(
                package="double_jearm_moveit_config",
                executable="fk_from_csv",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    {
                        "csv_path": LaunchConfiguration("csv_path"),
                        "arm_mode": LaunchConfiguration("arm_mode"),
                    },
                ],
            ),
        ]
    )
