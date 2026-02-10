cmake_minimum_required(VERSION 3.22)
project(double_jearm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

ament_package()

add_executable(joint_state_replay src/joint_state_replay.cpp)
ament_target_dependencies(
  joint_state_replay
  rclcpp
  sensor_msgs
  moveit_core
  moveit_ros_planning
  visualization_msgs
)

add_executable(fk_from_csv src/fk_from_csv.cpp)
ament_target_dependencies(
  fk_from_csv
  rclcpp
  ament_index_cpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(jitter_metrics src/jitter_metrics.cpp src/smoothness_metrics_utils.cpp)
ament_target_dependencies(
  jitter_metrics
  rclcpp
  moveit_core
  moveit_ros_planning
)

add_executable(jitter_demo src/jitter_demo.cpp src/smoothness_metrics_utils.cpp)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS joint_state_replay fk_from_csv jitter_metrics jitter_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
