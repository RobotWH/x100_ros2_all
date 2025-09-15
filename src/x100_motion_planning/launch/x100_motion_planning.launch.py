from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# 简化版：直接用 MoveItConfigsBuilder 生成需要的参数并启动节点
# 使用方法：ros2 launch x100_motion_planning x100_motion_planning.launch.py planning_group:=left_arm

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder('x100').to_moveit_configs()

    # print("== MoveIt Configs 内容 ==")
    # print(moveit_config)
        
    motion_node = Node(
        package='x100_motion_planning',
        executable='x100_motion_planning_node',
        name='x100_motion_planning_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ]
    )

    return LaunchDescription([
        motion_node
    ])
