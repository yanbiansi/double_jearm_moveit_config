Ubuntu 22.04
ROS2 Humble

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
# 进入 src 并克隆仓库
cd ~/ros2_ws/src
git clone https://github.com/yanbiansi/double_jearm_moveit_config

# 回到工作空间根目录
cd ~/ros2_ws

# 安装依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install --executor sequential

# source
source install/setup.bash

ros2 launch double_jearm_moveit_config jitter_metrics.launch.py csv_dir:=~/ros2_ws/src/double_jearm_moveit_config/csv arm_mode:=right



