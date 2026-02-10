Ubuntu 22.04
ROS2 Humble

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
# 进入 src 并克隆仓库
cd ~/ros2_ws/src
git clone https://github.com/yanbiansi/double_jearm_moveit_config.git

# 回到工作空间根目录
cd ~/ros2_ws

# 安装依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install --executor sequential

# source
source install/setup.bash

# 功能1：求平滑性指标
# 平滑指标launch文件，主要参数csv_dir，arm_mode
ros2 launch double_jearm_moveit_config jitter_metrics.launch.py csv_dir:=~/ros2_ws/src/double_jearm_moveit_config/csv arm_mode:=right

# 功能2：轨迹可视化
# 轨迹可视化复现launch文件
# 先启动rviz,robotmodel status error 正常
ros2 launch double_jearm_moveit_config rviz_robot_model.launch.py 

# 新终端，启动joint_state_replay.launch 主要参数：csv_path
ros2 launch double_jearm_moveit_config joint_state_replay.launch.py csv_path:="/home/wangyunhao/development/ros2_ws/src/double_jearm_moveit_config/test_joint_state_14.csv" arm_mode:="both"




