#include "x100_motion_planning/motion_planning.hpp"
#include <moveit/robot_state/conversions.hpp>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <sstream>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>// 替换原iterative_parabolic_time_parameterization
#include <chrono>
using namespace std::chrono_literals;

using namespace x100_motion_planning;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("x100_motion_planning::MotionPlanning");


MotionPlanning::MotionPlanning(rclcpp::Node::SharedPtr node, const std::string& planning_group)
  : node_(node),
    planning_group_(planning_group),
    robot_model_loader_(node_, "robot_description"),
    move_group_(node_, planning_group_)
{
  // constructor body intentionally small; heavy init in initialize()
  robot_model_ = robot_model_loader_.getModel();
  robot_state_.reset(new moveit::core::RobotState(robot_model_));
  joint_model_group_ = robot_state_->getJointModelGroup(planning_group_);
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
}

MotionPlanning::~MotionPlanning() = default;

bool MotionPlanning::initialize() 
{
  // configure planning scene and default state

  if (!joint_model_group_)
  {
    RCLCPP_ERROR(LOGGER, "JointModelGroup '%s' not found", planning_group_.c_str());
    return false;
  }
  // // 关节速度/加速度限制检查
  // {
  //   const auto& vars = joint_model_group_->getVariableNames();
  //   size_t missing_vel = 0, missing_acc = 0;
  //   RCLCPP_INFO(LOGGER, "[JointLimits] 检查规划组 '%s' 变量数=%zu", planning_group_.c_str(), vars.size());
  //   for (const auto& v : vars)
  //   {
  //     const auto& b = robot_model_->getVariableBounds(v);
  //     bool has_vel = b.velocity_bounded_ && b.max_velocity_ > 0.0;
  //     bool has_acc = b.acceleration_bounded_ && b.max_acceleration_ > 0.0;
  //     if (!has_vel) ++missing_vel;
  //     if (!has_acc) ++missing_acc;
  //     RCLCPP_INFO(LOGGER, "  变量 %-20s vel[%s] max=%.5f acc[%s] max=%.5f", v.c_str(),
  //                 has_vel?"Y":"N", b.max_velocity_, has_acc?"Y":"N", b.max_acceleration_);
  //   }
  //   if (missing_vel || missing_acc)
  //     RCLCPP_WARN(LOGGER, "[JointLimits] 缺失速度限制=%zu, 缺失加速度限制=%zu (可检查 joint_limits.yaml 或 SRDF)", missing_vel, missing_acc);
  //   else
  //     RCLCPP_INFO(LOGGER, "[JointLimits] 所有关节速度/加速度限制已加载");
  // }
  // planning_scene_->getCurrentStateNonConst().setToDefaultValues(joint_model_group_, "ready");
  display_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(node_, "base_link","x100_motion_trajectory", move_group_.getRobotModel()));
  visual_tools_->enableBatchPublishing();
  visual_tools_->deleteAllMarkers();  
  visual_tools_->loadRemoteControl();
  visual_tools_->trigger();

  if (!display_publisher_)
    display_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("display_planned_path", 10);
  return true;
}

moveit_visual_tools::MoveItVisualTools* MotionPlanning::visualTools()
{
  return visual_tools_.get();
}

moveit::planning_interface::MoveGroupInterface* MotionPlanning::moveGroup() {
  return &move_group_;
}

moveit::planning_interface::MoveGroupInterface::Plan MotionPlanning::planPoseGoal(const geometry_msgs::msg::PoseStamped& pose,std::string ee_link)
{
  // RCLCPP_INFO(LOGGER, "Current velocity scaling factor: %f", move_group_.getMaxVelocityScalingFactor());
  // RCLCPP_INFO(LOGGER, "Current acceleration scaling factor: %f", move_group_.getMaxAccelerationScalingFactor());
  if(ee_link.empty())
  {
    std::vector<std::string> link_names = move_group_.getLinkNames();
    ee_link = link_names.back();
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  move_group_.setStartStateToCurrentState();
  move_group_.setPoseTarget(pose, ee_link);
  move_group_.setPlanningTime(3.0);
  move_group_.setNumPlanningAttempts(3);

  if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "规划失败");
    return plan; // 空plan
  }
  if (plan.trajectory.joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(LOGGER, "规划轨迹为空");
    return plan; // 空plan
  }

  visual_tools_->deleteAllMarkers();
  visual_tools_->publishTrajectoryLine(plan.trajectory, robot_model_->getLinkModel(ee_link), joint_model_group_);
  visual_tools_->trigger();

  RCLCPP_WARN(LOGGER, "规划成功, 轨迹总点数: %zu", plan.trajectory.joint_trajectory.points.size());
  // // 发布 DisplayTrajectory
  // if (display_publisher_)
  // {
  //   moveit_msgs::msg::DisplayTrajectory msg;
  //   msg.model_id = robot_model_->getName();
  //   moveit::core::robotStateToRobotStateMsg(*move_group_.getCurrentState(), msg.trajectory_start);
  //   msg.trajectory.push_back(plan.trajectory);
  //   display_publisher_->publish(msg);
  // }
  return plan;
}

moveit::planning_interface::MoveGroupInterface::Plan MotionPlanning::planJointGoal(const std::vector<std::vector<double>>& way_points, std::string ee_link)
{
  if(ee_link.empty())
  {
    std::vector<std::string> link_names = move_group_.getLinkNames();
    ee_link = link_names.back();
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (way_points.empty())
  {
    RCLCPP_ERROR(LOGGER, "Joint multi-goal: 输入路点为空");
    return plan;
  }
  auto state = move_group_.getCurrentState(5);
  if (!state)
  {
    RCLCPP_ERROR(LOGGER, "Joint multi-goal: 无法获取当前状态");
    return plan;
  }
  const moveit::core::JointModelGroup* jmg = state->getJointModelGroup(planning_group_);
  if (!jmg)
  {
    RCLCPP_ERROR(LOGGER, "Joint multi-goal: JointModelGroup '%s' 不存在", planning_group_.c_str());
    return plan;
  }
  std::vector<double> start_positions;
  state->copyJointGroupPositions(jmg, start_positions);
  move_group_.setStartState(*state);
  move_group_.setPlanningTime(60.0);

  std::vector<moveit_msgs::msg::RobotTrajectory> partial_trajs;
  partial_trajs.reserve(way_points.size());
  bool ok_all = true;
  for (size_t i = 0; i < way_points.size(); ++i)
  {
    move_group_.setJointValueTarget(way_points[i]);
    moveit::planning_interface::MoveGroupInterface::Plan partial;
    bool ok = (move_group_.plan(partial) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok || partial.trajectory.joint_trajectory.points.empty())
    {
      RCLCPP_ERROR(LOGGER, "Joint multi-goal: 第 %zu 段规划失败", i);
      ok_all = false;
      break;
    }
    RCLCPP_WARN(LOGGER, "Joint multi-goal: 段 %zu 规划成功, 轨迹总点数=%zu", i, partial.trajectory.joint_trajectory.points.size());
    partial_trajs.push_back(partial.trajectory);
    // 更新起始状态为该段终点
    state->setJointGroupPositions(jmg, way_points[i]);
    state->update();
    move_group_.setStartState(*state);
  }

  // 复原起始状态
  state->setJointGroupPositions(jmg, start_positions);
  state->update();
  move_group_.setStartState(*state);

  if (!ok_all || partial_trajs.empty())
  {
    RCLCPP_ERROR(LOGGER, "Joint multi-goal: 组合失败");
    return plan;
  }

  // 拼接轨迹
  trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
  robot_trajectory::RobotTrajectory combined(state->getRobotModel(), planning_group_);
  robot_trajectory::RobotTrajectory tmp(state->getRobotModel(), planning_group_);
  combined.setRobotTrajectoryMsg(*state, partial_trajs[0]);
  for (size_t i = 1; i < partial_trajs.size(); ++i)
  {
    tmp.setRobotTrajectoryMsg(combined.getLastWayPoint(), partial_trajs[i]);
    combined.append(tmp, 0.0, 1.0); // 保留时间顺序; scale=1.0
    tmp.clear();
  }

  // 时间参数化
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(combined, 
                              move_group_.getMaxVelocityScalingFactor(), 
                              move_group_.getMaxAccelerationScalingFactor()))
  {
    RCLCPP_ERROR(LOGGER, "Joint multi-goal: 时间参数化失败");
    return plan;
  }
  // 输出到 plan
  combined.getRobotTrajectoryMsg(plan.trajectory);
  plan.start_state = moveit_msgs::msg::RobotState();
  RCLCPP_WARN(LOGGER, "Joint multi-goal: 轨迹总段数=%zu, 合并后轨迹总点数=%zu", partial_trajs.size(), plan.trajectory.joint_trajectory.points.size());

  // 可视化 
  visual_tools_->deleteAllMarkers();  
  visual_tools_->publishTrajectoryLine(plan.trajectory, robot_model_->getLinkModel(ee_link), joint_model_group_);
  visual_tools_->trigger();

  if (display_publisher_)
  {
    moveit_msgs::msg::DisplayTrajectory msg;
    msg.model_id = robot_model_->getName();
    moveit::core::robotStateToRobotStateMsg(*move_group_.getCurrentState(), msg.trajectory_start);
    msg.trajectory.push_back(plan.trajectory);
    display_publisher_->publish(msg);
  }
  return plan;
}

bool MotionPlanning::executePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  if (plan.trajectory.joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(LOGGER, "执行失败: 轨迹为空");
    return false;
  }
  auto ret = move_group_.execute(plan);
  if (ret != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "执行失败: 执行过程中出现错误 (%d)", ret.val);
    return false;
  }
  RCLCPP_WARN(LOGGER, "轨迹执行成功");
  return true;
}
