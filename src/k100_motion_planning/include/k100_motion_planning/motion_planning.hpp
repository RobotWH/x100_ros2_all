#ifndef k100_MOTION_PLANNING__MOTION_PLANNING_HPP_
#define k100_MOTION_PLANNING__MOTION_PLANNING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sensor_msgs/msg/joy.hpp>  // 兼容RViz面板发布为Joy的情况
#include <sstream>
#include <string>
#include <vector>
#include <memory>

namespace k100_motion_planning {

class MotionPlanning {
public:
  /*!\brief 构造函数
   *  @param node 共享 ROS2 节点指针
   *  @param planning_group 规划组名称 (对应 SRDF 中的 group)
   */
  explicit MotionPlanning(rclcpp::Node::SharedPtr node, const std::string& planning_group = "left_arm");
  /*!\brief 析构函数 */
  ~MotionPlanning();

  /*!\brief 初始化内部资源 (模型 / 可视化 / 发布器)
   *  @return true 成功; false 失败
   */
  bool initialize();

  /*!\brief 规划一个末端姿态/位姿目标 (笛卡尔空间)
   *  @param pose 目标位姿 (header.frame_id 应与规划参考坐标系一致)
   *  @param ee_link 末端执行器链接, 为空则自动取默认末端
   *  @return 规划结果 Plan (若失败其中 trajectory 为空)
   */
  moveit::planning_interface::MoveGroupInterface::Plan planPoseGoal(const geometry_msgs::msg::PoseStamped& pose,
                                                                    std::string ee_link = "");
  /*!\brief 按顺序规划多个关节空间路点并合并为一个轨迹
   *  @param way_points 关节路点数组(每个内部 vector 对应一组关节角)
   *  @param ee_link 末端执行器链接(可选, 仅用于可视化)
   *  @return 合并后的轨迹 Plan (失败 trajectory 为空)
   */
  moveit::planning_interface::MoveGroupInterface::Plan planJointGoal(const std::vector<std::vector<double>>& way_points,
                                                                     std::string ee_link = "");

  /*!\brief 执行已规划轨迹
   *  @param plan 规划结果
   *  @return true 执行成功; false 执行失败或轨迹为空
   */
  bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
 
  /*!\brief 获取可视化工具指针 (用于发布 Marker / 轨迹)
   *  @return MoveItVisualTools 原始指针 (可能为空, 使用前检查)
   */
  moveit_visual_tools::MoveItVisualTools* visualTools();

  /*!\brief 获取内部 MoveGroupInterface 指针 (便于外部直接设置目标等)
   *  @return MoveGroupInterface 指针
   */
  moveit::planning_interface::MoveGroupInterface* moveGroup();
  
  /*!\brief 逆运动学求解 (返回首个可行关节解)
   *  @param pose 末端目标位姿
   *  @param ee_link 末端执行器链接 (为空自动推断)
   *  @param timeout 单次 IK 求解超时时间(秒)
   *  @param attempts 尝试次数 (随机扰动初始种子)
   *  @return 成功返回关节角数组; 失败返回空 vector
   */
  std::vector<double> computeIKArray(const geometry_msgs::msg::PoseStamped& pose,
                                     std::string ee_link = "",
                                     double timeout = 0.2,
                                     unsigned int attempts = 5);

  /*!\brief 获取当前规划组关节角
   *  @param timeout 状态获取超时(秒)
   *  @return 关节角数组; 失败为空
   */
  std::vector<double> getCurrentJointPositions(double timeout = 1.0) const;

  
  /** \brief 直接返回指定 link 位姿（失败时 frame_id 为空）
   *  @param link_name 链接名称
   *  @param timeout 获取当前状态超时
   *  @return PoseStamped (失败时 header.frame_id 为空字符串)
   */
  geometry_msgs::msg::PoseStamped getLinkPoseDirect(const std::string& link_name,
                                                    double timeout = 1.0) const;

  /** \brief 载入 STL/OBJ 网格并作为障碍物加入场景
   *  @param object_id 唯一ID
   *  @param mesh_path 绝对路径(/home/..)、file:// 或 package:/// 资源URI
   *  @param pose 放置位姿(参考=规划坐标系)
   *  @param frame_id 参考坐标系
   *  @param scale_x 缩放系数
   *  @param scale_y 缩放系数
   *  @param scale_z 缩放系数
   
   *  @return true 成功 false 失败
   *  */
  bool addCollisionMesh(const std::string& object_id,
                        const std::string& mesh_path,
                        const geometry_msgs::msg::Pose& pose,
                        const std::string& frame_id = "base_link",
                        double scale_x = 1.0,
                        double scale_y = 1.0,
                        double scale_z = 1.0);

  /** \brief 添加基本形状障碍物 (BOX / SPHERE / CYLINDER / CONE)
   *  @param object_id 障碍物唯一ID
   *  @param primitive 已配置 type 与 dimensions 的 SolidPrimitive
   *  @param pose 物体在参考坐标系下的位姿
   *  @param frame_id 参考坐标系 (为空则使用规划坐标系)
   *  @return true 添加成功, false 失败
   */
  bool addCollisionPrimitive(const std::string& object_id,
                             const shape_msgs::msg::SolidPrimitive& primitive,
                             const geometry_msgs::msg::Pose& pose,
                             const std::string& frame_id = "base_link");

  /** \brief 删除指定碰撞对象 (若不存在则返回 true)
   *  @param object_id 碰撞对象ID
   *  @return true 删除成功或对象原本不存在; false 删除失败
   */
  bool removeCollisionObject(const std::string& object_id);

  /** \brief 返回当前规划场景中所有碰撞对象名称 */
  std::vector<std::string> listCollisionObjects();

  /** \brief 检查多组关节角是否与当前场景发生碰撞 (全部无碰撞返回 true)
   *  @param joint_positions 规划组的关节角数组(每组长度需与该规划组变量数一致)
   *  @return true 全部无碰撞; false 有碰撞或检查失败
   */
  bool checkJointCollision(const std::vector<std::vector<double>>& joint_positions);

  /** \brief 设置两个物体在 AllowedCollisionMatrix 中是否允许发生碰撞 (默认允许)
   *  @param object_a 物体A ID
   *  @param object_b 物体B ID
   *  @param allow true 允许碰撞; false 禁止碰撞
   *  @return true 设置成功; false 失败
   */
  
  bool allowCollisionBetween(const std::string& object_a, const std::string& object_b, bool allow = true);

private:
  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  moveit::core::RobotModelPtr          robot_model_;
  moveit::core::RobotStatePtr          robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_ {nullptr};
  planning_scene::PlanningScenePtr     planning_scene_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  int stop_button_index_ = 4; // 默认按钮索引
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_; // STOP 按钮订阅
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher_;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_; // 场景监视器(用户请求命名)
};

} // namespace k100_motion_planning

#endif // k100_MOTION_PLANNING__MOTION_PLANNING_HPP_