from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(robot_name="D_JEARM",package_name="double_jearm_moveit_config")
        .robot_description(file_path="config/D_JEARM.urdf.xacro")
        .robot_description_semantic(file_path="config/D_JEARM.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    pkg_share = get_package_share_directory("double_jearm_moveit_config")
    rviz_config = os.path.join(pkg_share, "config", "1.rviz")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )


    return LaunchDescription(
        [
            robot_state_publisher,
            # joint_state_publisher,
            rviz,
        ]
    )
