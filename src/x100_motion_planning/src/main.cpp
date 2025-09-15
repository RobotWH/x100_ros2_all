#include <rclcpp/rclcpp.hpp>
#include <string> // 添加用于 std::string
#include <vector> // 添加用于 std::vector
#include <thread> // 补充线程头文件
#include "x100_motion_planning/motion_planning.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("x100_motion_planning_node", node_options);

    // run executor in background
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // 仅运行单臂示例（默认 left_arm），可通过参数覆盖 move_group
    std::string move_group = node->declare_parameter<std::string>("move_group", "left_arm");

    RCLCPP_ERROR(node->get_logger(), "Using move group: '%s'", move_group.c_str());
    x100_motion_planning::MotionPlanning planner(node, move_group);

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


    // 设置速度/加速度缩放因子
    planner.moveGroup()->setMaxVelocityScalingFactor(0.5);
    planner.moveGroup()->setMaxAccelerationScalingFactor(0.5);
    ////////////笛卡尔空间规划///////////
    geometry_msgs::msg::PoseStamped pose;
    // NOTE: 根据你的机器人调整 frame_id，例如 "base_link" 或 各臂对应的 link
    pose.header.frame_id = "base_link"; 
    pose.pose.position.x = 0.19;
    pose.pose.position.y = 0.21;
    pose.pose.position.z = 0.70;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = -0.625900;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.78;

    // planner.visualTools()->publishText(Eigen::Isometry3d::Identity(), (std::string("Motion Planning Demo: ") + move_group).c_str(), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    planner.visualTools()->trigger();
    planner.visualTools()->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    auto pose_plan = planner.planPoseGoal(pose);
    planner.visualTools()->prompt("Press 'next' to execute pose plan");
    planner.executePlan(pose_plan);
    planner.visualTools()->prompt("Press 'next' to plan a joint-space goal");


    ////////////关节空间规划///////////
    std::vector<std::vector<double>> joint_values = { { 0.2, 0.1, -0.087, -1.3, 0.38, -0.50, -0.334 }, 
                                                        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    auto joint_plan = planner.planJointGoal(joint_values);
    
    planner.visualTools()->prompt("Press 'next' to execute joint plan");
    planner.executePlan(joint_plan);
    planner.visualTools()->prompt((std::string("Demo for ") + move_group + " finished. Press 'next' to continue").c_str());

    rclcpp::shutdown();
    return 0;
}
