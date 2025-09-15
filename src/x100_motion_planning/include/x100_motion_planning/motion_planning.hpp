#ifndef X100_MOTION_PLANNING__MOTION_PLANNING_HPP_
#define X100_MOTION_PLANNING__MOTION_PLANNING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>
#include <memory>

namespace x100_motion_planning {

class MotionPlanning {
public:
  explicit MotionPlanning(rclcpp::Node::SharedPtr node, const std::string& planning_group = "left_arm");
  ~MotionPlanning();

  bool initialize();

  // 笛卡尔/姿态目标规划
  moveit::planning_interface::MoveGroupInterface::Plan planPoseGoal(std::string planning_group,
                                                                    std::string ee_link,
                                                                    const geometry_msgs::msg::PoseStamped& pose);
  // 多关节路点顺序规划并合并（带时间参数化）
  moveit::planning_interface::MoveGroupInterface::Plan planJointGoal(const std::vector<std::vector<double>>& way_points,
                                                                     const std::string& ee_link = "");

  bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan);

  // 可视化工具访问
  moveit_visual_tools::MoveItVisualTools* visualTools();

private:
  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;

  robot_model_loader::RobotModelLoader robot_model_loader_;
  moveit::core::RobotModelPtr          robot_model_;
  moveit::core::RobotStatePtr          robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_ {nullptr};
  planning_scene::PlanningScenePtr     planning_scene_;

  moveit::planning_interface::MoveGroupInterface move_group_;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher_;
};

} // namespace x100_motion_planning

#endif // X100_MOTION_PLANNING__MOTION_PLANNING_HPP_
