#include "k100_motion_planning/motion_planning.hpp"
#include <sstream>
#include <chrono>
#include <algorithm>
#include <moveit_msgs/msg/planning_scene.hpp>
using namespace std::chrono_literals;

using namespace k100_motion_planning;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("k100_motion_planning::MotionPlanning");


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
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(node_, "base_link","k100_motion_trajectory", move_group_.getRobotModel()));
  visual_tools_->enableBatchPublishing();
  visual_tools_->deleteAllMarkers();  
  visual_tools_->loadRemoteControl();
  visual_tools_->trigger();

  if (!display_publisher_)
    display_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("display_planned_path", 10);


  // Joy 按钮方式 STOP (默认使用第4号索引按钮, 参数可调)
  stop_button_index_ = node_->declare_parameter<int>("stop_button_index", 4); // 使用成员变量, 与示例中 buttons[4]==1 对应
  joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "rviz_visual_tools_gui", 10,
      [this](const sensor_msgs::msg::Joy::SharedPtr msg)
      {
        if (stop_button_index_ >= 0 && stop_button_index_ < static_cast<int>(msg->buttons.size()))
        {
          if (msg->buttons[stop_button_index_] != 0)
          {
            static bool exiting = false;
            if (!exiting) {
              exiting = true;
              RCLCPP_WARN(LOGGER, "Joy STOP 按钮(索引=%d) 被按下, 退出节点...", stop_button_index_);
              rclcpp::shutdown();
              std::exit(0);
            }
          }
        }
      });

  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
  if (!planning_scene_monitor_)
  {
    RCLCPP_ERROR(LOGGER, "初始化 PlanningSceneMonitor 失败");
  }
  else
  {
    planning_scene_monitor_->providePlanningSceneService();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    RCLCPP_INFO(LOGGER, "PlanningSceneMonitor 已启动");
  }

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
  planning_scene_monitor_->requestPlanningSceneState();

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
  planning_scene_monitor_->requestPlanningSceneState();
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
      return plan;
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

std::vector<double> MotionPlanning::computeIKArray(const geometry_msgs::msg::PoseStamped& pose,
                                     std::string ee_link,
                                     double timeout,
                                     unsigned int attempts)
{
  if (ee_link.empty())
  {
    ee_link = move_group_.getEndEffectorLink();
    if (ee_link.empty())
    {
      const auto links = move_group_.getLinkNames();
      if (!links.empty()) ee_link = links.back();
    }
  }
  auto current = move_group_.getCurrentState(2.0);
  if (!current)
  {
    RCLCPP_ERROR(LOGGER, "computeIKArray: 获取当前状态失败");
    return {};
  }
  const auto* jmg = current->getJointModelGroup(planning_group_);
  if (!jmg)
  {
    RCLCPP_ERROR(LOGGER, "computeIKArray: JointModelGroup '%s' 未找到", planning_group_.c_str());
    return {};
  }
  std::vector<double> joints;
  bool solved = false;
  for (unsigned int i = 0; i < attempts && !solved; ++i)
  {
    moveit::core::RobotState trial = *current;
    solved = trial.setFromIK(jmg, pose.pose, ee_link, timeout);
    if (solved)
    {
      if (!trial.satisfiesBounds(jmg))
        trial.enforceBounds(jmg);
      trial.copyJointGroupPositions(jmg, joints);
    }
  }
  if (!solved)
    RCLCPP_WARN(LOGGER, "computeIKArray: 逆解失败 ee_link=%s", ee_link.c_str());
  return joints;
}


std::vector<double> MotionPlanning::getCurrentJointPositions(double timeout) const
{
  std::vector<double> vals;
  auto state = move_group_.getCurrentState(timeout);
  if (!state)
  {
    RCLCPP_ERROR(LOGGER, "getCurrentJointPositions: 获取当前状态失败");
    return vals;
  }
  const auto* jmg = state->getJointModelGroup(planning_group_);
  if (!jmg)
  {
    RCLCPP_ERROR(LOGGER, "getCurrentJointPositions: JointModelGroup '%s' 不存在", planning_group_.c_str());
    return vals;
  }
  state->copyJointGroupPositions(jmg, vals);
  return vals;
}

geometry_msgs::msg::PoseStamped MotionPlanning::getLinkPoseDirect(const std::string& link_name,
                                                    double timeout) const
{
  geometry_msgs::msg::PoseStamped ps;
  auto state = move_group_.getCurrentState(timeout);
  if (!state)
  {
    RCLCPP_ERROR(LOGGER, "getLinkPoseDirect: 获取当前状态失败");
    return ps;
  }
  const moveit::core::LinkModel* link = state->getLinkModel(link_name);
  if (!link)
  {
    RCLCPP_ERROR(LOGGER, "getLinkPoseDirect: link '%s' 不存在", link_name.c_str());
    return ps;
  }
  const auto& tf = state->getGlobalLinkTransform(link);
  ps.header.stamp = node_->now();
  ps.header.frame_id = move_group_.getPlanningFrame();
  ps.pose.position.x = tf.translation().x();
  ps.pose.position.y = tf.translation().y();
  ps.pose.position.z = tf.translation().z();
  Eigen::Quaterniond q(tf.rotation());
  ps.pose.orientation.x = q.x();
  ps.pose.orientation.y = q.y();
  ps.pose.orientation.z = q.z();
  ps.pose.orientation.w = q.w();
  return ps;
}

bool MotionPlanning::addCollisionMesh(const std::string& object_id,
                        const std::string& mesh_path,
                        const geometry_msgs::msg::Pose& pose,
                        const std::string& frame_id,
                        double scale_x,
                        double scale_y,
                        double scale_z)
{
  shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);
  if (!m)
  {
    RCLCPP_ERROR(LOGGER, "addCollisionMesh: 加载网格失败 '%s'", mesh_path.c_str());
    return false;
  }
  for (unsigned int i = 0; i < m->vertex_count; ++i)
  {
    m->vertices[3*i + 0] *= scale_x;
    m->vertices[3*i + 1] *= scale_y;
    m->vertices[3*i + 2] *= scale_z;
  }
  shape_msgs::msg::Mesh mesh_msg;
  shapes::ShapeMsg mesh_msg_variant;
  shapes::constructMsgFromShape(m, mesh_msg_variant);
  mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_variant);
  delete m;

  moveit_msgs::msg::CollisionObject co;
  co.id = object_id;
  co.header.frame_id = frame_id;
  co.meshes.push_back(mesh_msg);
  co.mesh_poses.push_back(pose);
  co.operation = moveit_msgs::msg::CollisionObject::ADD;

  if (planning_scene_interface_.applyCollisionObject(co))
  {
    RCLCPP_INFO(LOGGER, "addCollisionMesh: 添加成功 id=%s", object_id.c_str());
    return true;
  }
  RCLCPP_ERROR(LOGGER, "addCollisionMesh: 应用到场景失败 id=%s", object_id.c_str());
  planning_scene_monitor_->requestPlanningSceneState();

  return false;
}

bool MotionPlanning::addCollisionPrimitive(const std::string& object_id,
                             const shape_msgs::msg::SolidPrimitive& primitive,
                             const geometry_msgs::msg::Pose& pose,
                             const std::string& frame_id)
{
  if (primitive.type == 0) {
    RCLCPP_ERROR(LOGGER, "addCollisionPrimitive: primitive.type 未设置");
    return false;
  }
  moveit_msgs::msg::CollisionObject co;
  co.id = object_id;
  co.header.frame_id = frame_id;
  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(pose);
  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  if (planning_scene_interface_.applyCollisionObject(co))
  {
    RCLCPP_INFO(LOGGER, "addCollisionPrimitive: 添加成功 id=%s", object_id.c_str());
    return true;
  }
  RCLCPP_ERROR(LOGGER, "addCollisionPrimitive: 失败 id=%s", object_id.c_str());
  return false;
}

bool MotionPlanning::removeCollisionObject(const std::string& object_id)
{
  if (object_id.empty())
  {
    RCLCPP_ERROR(LOGGER, "removeCollisionObject: object_id 为空");
    return false;
  }
  auto names_before = planning_scene_interface_.getKnownObjectNames();
  bool existed = std::find(names_before.begin(), names_before.end(), object_id) != names_before.end();
  if (!existed)
  {
    RCLCPP_WARN(LOGGER, "removeCollisionObject: 对象 '%s' 不存在, 视为已删除", object_id.c_str());
    return true;
  }
  planning_scene_interface_.removeCollisionObjects({object_id});
  rclcpp::sleep_for(std::chrono::milliseconds(50));
  auto names_after = planning_scene_interface_.getKnownObjectNames();
  bool still_here = std::find(names_after.begin(), names_after.end(), object_id) != names_after.end();
  if (still_here)
  {
    RCLCPP_ERROR(LOGGER, "removeCollisionObject: 删除失败 '%s'", object_id.c_str());
    return false;
  }
  RCLCPP_INFO(LOGGER, "removeCollisionObject: 已删除 '%s'", object_id.c_str());
  planning_scene_monitor_->requestPlanningSceneState();
  return true;
}

std::vector<std::string> MotionPlanning::listCollisionObjects()
{
  planning_scene_monitor_->requestPlanningSceneState();
  return planning_scene_interface_.getKnownObjectNames();
}

bool MotionPlanning::checkJointCollision(const std::vector<std::vector<double>>& joint_positions)
{
  if (joint_positions.empty())
  {
    RCLCPP_WARN(LOGGER, "checkJointCollision: 输入关节组为空");
    return true; // 视为空任务
  }
  planning_scene_monitor_->requestPlanningSceneState();
  planning_scene::PlanningScenePtr ps =  planning_scene_monitor_->getPlanningScene() ;

  std::vector<double> backup_positions;
  moveit::core::RobotState cur_state(ps->getCurrentState());
  cur_state.copyJointGroupPositions(joint_model_group_, backup_positions);

  // const auto& world = ps_monitor->getWorld();
  // auto ids = world->getObjectIds();
  // RCLCPP_INFO(LOGGER, "checkJointCollision: 当前障碍物数量=%zu, 待检测组数=%zu", ids.size(), joint_positions.size());
  // size_t printed = 0;
  // for (const auto& id : ids)
  // {
  //   if (printed >= 30) { RCLCPP_INFO(LOGGER, "  ...其余障碍物省略"); break; }
  //   auto obj = world->getObject(id); if(!obj) continue;
  //   RCLCPP_INFO(LOGGER, "  obstacle[%zu]: id=%s shapes=%zu", printed, id.c_str(), obj->shapes_.size());
  //   ++printed;
  // }

  // 循环检测
  bool have_collision = false;
  for (size_t idx = 0; idx < joint_positions.size(); ++idx)
  {
    const auto& jp = joint_positions[idx];
    if (!joint_model_group_)
    {
      RCLCPP_ERROR(LOGGER, "checkJointCollision: JointModelGroup 不存在");
      break;
    }
    if (jp.size() != joint_model_group_->getVariableCount())
    {
      RCLCPP_ERROR(LOGGER, "checkJointCollision: 第%zu组关节数不匹配 输入=%zu 需要=%u", idx, jp.size(), joint_model_group_->getVariableCount());
      return false;
    }
    
    cur_state.setJointGroupPositions(joint_model_group_, jp);
    cur_state.update();

    collision_detection::CollisionRequest req; 
    collision_detection::CollisionResult res;
    req.contacts = true; 
    req.max_contacts = 200; 
    req.max_contacts_per_pair = 5; 
    req.group_name = planning_group_;
    ps->checkCollision(req, res, cur_state, ps->getAllowedCollisionMatrix());
    bool collided_env = res.collision;


    if (collided_env && !res.contacts.empty())
    {
        size_t shown = 0;
        for (const auto& cpair : res.contacts)
        {
          RCLCPP_ERROR(LOGGER, " 第%zu组规划点,发生碰撞,碰撞对为: %s <-> %s ", idx, cpair.first.first.c_str(), cpair.first.second.c_str());
          if (++shown >= 20) 
          { 
            RCLCPP_ERROR(LOGGER, "  ...其余碰撞对省略"); break; 
          }
        }
        have_collision = true;
    }
    else
    {
      RCLCPP_INFO(LOGGER, "checkJointCollision: 第%zu组规划点, 无碰撞", idx);
    }
  }

  // 全部检查后恢复状态
  if (!backup_positions.empty())
  {
    cur_state.setJointGroupPositions(joint_model_group_, backup_positions);
    cur_state.update();
  }
  return !have_collision;
}

bool MotionPlanning::allowCollisionBetween(const std::string& object_a, const std::string& object_b, bool allow)
{
  if (object_a.empty() || object_b.empty())
  {
    RCLCPP_ERROR(LOGGER, "allowCollisionBetween: 物体名称为空");
    return false;
  }
  planning_scene_monitor_->requestPlanningSceneState();
  auto ps =planning_scene_monitor_->getPlanningScene();
  auto& acm = ps->getAllowedCollisionMatrixNonConst();
  acm.setEntry(object_a, object_b, allow);
  moveit_msgs::msg::PlanningScene scene_msg;
  ps->getPlanningSceneMsg(scene_msg);
  planning_scene_interface_.applyPlanningScene(scene_msg);

  RCLCPP_INFO(LOGGER, "allowCollisionBetween: %s <-> %s 设置为 %s", object_a.c_str(), object_b.c_str(), allow?"允许":"禁止");
  return true;
}