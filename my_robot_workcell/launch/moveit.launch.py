import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # FIXED: Explicitly defined the robot_name and the package_name so it doesn't guess!
    moveit_config = (
        MoveItConfigsBuilder("my_inverted_ur", package_name="my_robot_workcell")
        .robot_description(file_path="urdf/inverted_ur.urdf.xacro")
        .robot_description_semantic(file_path="config/my_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("my_robot_workcell"), "rviz", "view_robot.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([move_group_node, rviz_node])