#include <rclcpp/rclcpp.hpp>
#include <string> 
#include <vector> 
#include <thread> 
#include <sstream> 
#include "k100_motion_planning/motion_planning.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("k100_motion_planning_node", node_options);

    // run executor in background
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // 仅运行单臂示例（默认 left_arm），可通过参数覆盖 move_group
    std::string move_group = node->declare_parameter<std::string>("move_group", "left_arm");

    RCLCPP_ERROR(node->get_logger(), "Using move group: '%s'", move_group.c_str());
    k100_motion_planning::MotionPlanning planner(node, move_group);

    if (!planner.initialize())
    {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "MotionPlanning initialization failed for group '%s'", move_group.c_str());
        rclcpp::shutdown();
        return -1;
    }

    // shoulder_pitch_link
    // └── shoulder_roll_link
    //    └── shoulder_yaw_link
    //       └── elbow_link
    //             └── wrist_roll_link
    //                   └── wrist_pitch_link
    //                         └── wrist_yaw_link


    // 设置速度/加速度缩放因子/////////////////////////////////
    planner.moveGroup()->setMaxVelocityScalingFactor(0.5);
    planner.moveGroup()->setMaxAccelerationScalingFactor(0.5);
    ////////////笛卡尔空间规划/////////////////////////////////////////////////////////////////
    // geometry_msgs::msg::PoseStamped pose;
    // // NOTE: 根据你的机器人调整 frame_id，例如 "base_link" 或 各臂对应的 link
    // pose.header.frame_id = "base_link"; 
    // pose.pose.position.x = 0.19;
    // pose.pose.position.y = 0.21;
    // pose.pose.position.z = 0.70;
    // pose.pose.orientation.x = 0.0;
    // pose.pose.orientation.y = -0.625900;
    // pose.pose.orientation.z = 0.0;
    // pose.pose.orientation.w = 0.78;

    // // planner.visualTools()->publishText(Eigen::Isometry3d::Identity(), (std::string("Motion Planning Demo: ") + move_group).c_str(), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    // planner.visualTools()->trigger();
    // planner.visualTools()->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // auto pose_plan = planner.planPoseGoal(pose);
    // planner.visualTools()->prompt("Press 'next' to execute pose plan");
    // planner.executePlan(pose_plan);
    // planner.visualTools()->prompt("Press 'next' to computeIK");

    //////////////////////////////////////////////////获取机器人当前关节角///////////////////////////////////////////////////////////////////
    // auto joint_positions = planner.getCurrentJointPositions();

    // std::ostringstream oss;
    // oss << "Current joints (" << move_group << "):";
    // for (size_t i = 0; i < joint_positions.size(); ++i)
    //     oss << " J" << i << "=" << joint_positions[i];
    // RCLCPP_INFO(node->get_logger(), "%s", oss.str().c_str());
    
    // std::vector<double> ik_solution = planner.computeIKArray(pose);

    // oss.str("");
    // oss.clear();
    // oss << "IK joints (" << move_group << "):";
    // for (size_t i = 0; i < ik_solution.size(); ++i)
    //     oss << " J" << i << "=" << ik_solution[i];
    // RCLCPP_INFO(node->get_logger(), "%s", oss.str().c_str());

    // auto ik_plan = planner.planJointGoal({ik_solution});
    // planner.executePlan(ik_plan);
    // planner.visualTools()->prompt("Press 'next' to execute get link pose");

    // ////////////////////////////////////获取指定 link 位姿////////////////////////////////////////////////
    // geometry_msgs::msg::PoseStamped wrist_pose = planner.getLinkPoseDirect("left_wrist_yaw_link");
    // RCLCPP_INFO(node->get_logger(), "Wrist Pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] ", 
    //             wrist_pose.pose.position.x, wrist_pose.pose.position.y, wrist_pose.pose.position.z,
    //             wrist_pose.pose.orientation.w, wrist_pose.pose.orientation.x,
    //             wrist_pose.pose.orientation.y, wrist_pose.pose.orientation.z);
    // planner.visualTools()->prompt("Press 'next' to execute multi joints plan");
    
    //////////////////////////////////////////////添加指定模型障碍物////////////////////////////////////////////
    planner.visualTools()->prompt("Press 'next' to add door");
    auto path = "package://k100_description/meshes/mesh_door/door.stl";
    // RCLCPP_ERROR(node->get_logger(), "Path: '%s'", path);
    geometry_msgs::msg::Pose door_pose; // 不能用聚合指定初始化, 逐字段赋值
    tf2::Quaternion q;
    q.setRPY(1.57, 0.0, 3.14);  // 按 RPY 赋值
    door_pose.position.x = 0.43;
    door_pose.position.y = 1.32;
    door_pose.position.z = 0.0;
    door_pose.orientation.x = q.x();
    door_pose.orientation.y = q.y();
    door_pose.orientation.z = q.z(); 
    door_pose.orientation.w = q.w();
    planner.addCollisionMesh("door", path, door_pose,"base_link",0.001,0.001,0.001);



    /////////////////////////////////////添加指定形状障碍物  CYLINDER圆柱,SPHERE球体，CONE圆锥///////////////////////
    // planner.visualTools()->prompt("Press 'next' to add BOX");
    // shape_msgs::msg::SolidPrimitive prim;
    // prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    // prim.dimensions.resize(3);
    // prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.1;
    // prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.2;
    // prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.3;

    //     prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    //     prim.dimensions.resize(2);
    //     prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.4;
    //     prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.15;

    //     prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    //     prim.dimensions.resize(1);
    //     prim.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = 0.1;

    //     prim.type = shape_msgs::msg::SolidPrimitive::CONE;
    //     prim.dimensions.resize(2);
    //     prim.dimensions[shape_msgs::msg::SolidPrimitive::CONE_HEIGHT] = 0.4;
    //     prim.dimensions[shape_msgs::msg::SolidPrimitive::CONE_RADIUS]  = 0.1;
    // planner.addCollisionPrimitive("box1", prim, door_pose);


    ////////////////////////////////删除指定障碍物////////////////////////////////////////
    // planner.visualTools()->prompt("Press 'next' to remove BOX");
    // auto collision_objects = planner.listCollisionObjects();
    // for (const auto& obj : collision_objects) {
    //     RCLCPP_INFO(node->get_logger(), "Collision Object: %s", obj.c_str());
    // }

    // planner.removeCollisionObject("box1");

    //////////////////////////判断是否碰撞////////////////////////////////
    // planner.visualTools()->prompt("Press 'next' to check collision");
    // std::vector<std::vector<double>>  check_joint_positions = {{-0.5585,-0.0175,-1.4661,-0.4538,1.4486,-0.4538,0.3142},
    //                                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    // bool collision_free = planner.checkJointCollision(check_joint_positions);


    ////////////////////////////允许两个物体碰撞////////////////////////////////
    // planner.allowCollisionBetween("door", "L_hand_base_link", true);
    // planner.allowCollisionBetween("door", "L_pinky_proximal", true);
    // planner.allowCollisionBetween("door", "L_ring_proximal", true);
    // planner.allowCollisionBetween("door", "L_thumb_proximal", true);

    /////////////////////////////////////////////关节空间规划////////////////////////////////////////////
    // std::vector<std::vector<double>> joint_values = { { 0.2, 0.1, -0.087, -1.3, 0.38, -0.50, -0.334 }, 
    //                                                     { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    std::vector<std::vector<double>> joint_positions = {{-0.7854, -0.5934, 0.6109, 1.0297, 0.7854, -0.0175, -0.2443},
                                                        {-1.1519, -0.2967, 0.7156, 0.2269, 0.8727, 0.2269, 0.0175}};
    
    if (planner.checkJointCollision(joint_positions))
    {
        auto joint_plan = planner.planJointGoal(joint_positions);
        planner.visualTools()->prompt("Press 'next' to execute joint plan");
        planner.executePlan(joint_plan);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Joint positions are in collision, cannot execute plan.");
    }

    //////////////////////////////////////////完成示例，退出////////////////////////////////////////////////
    planner.visualTools()->prompt("Press 'next' to finish ");

    rclcpp::shutdown();
    return 0; 
} 
     