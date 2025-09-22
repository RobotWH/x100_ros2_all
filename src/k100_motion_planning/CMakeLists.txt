cmake_minimum_required(VERSION 3.5)
project(k100_motion_planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono system filesystem)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  src/motion_planning.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp pluginlib 
  moveit_core 
  moveit_ros_planning_interface 
  moveit_visual_tools 
  moveit_msgs 
  geometry_msgs
)

add_executable(k100_motion_planning_node src/main.cpp)
target_link_libraries(k100_motion_planning_node ${PROJECT_NAME}_lib)
ament_target_dependencies(k100_motion_planning_node 
rclcpp moveit_core 
moveit_ros_planning_interface 
moveit_visual_tools 
moveit_msgs 
geometry_msgs 
pluginlib)

install(TARGETS
  ${PROJECT_NAME}_lib
  k100_motion_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
