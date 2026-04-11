import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Show Gazebo where the PickNik Robotiq 3D mesh files are
    robotiq_share_dir = get_package_share_directory('robotiq_description')
    gazebo_models_path = os.path.abspath(os.path.join(robotiq_share_dir, '..'))

    # Load the core UR Simulator
    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gazebo"),
                "launch",
                "ur_sim_control.launch.py"
            ])
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "description_package": "my_robot_workcell",
            "description_file": "inverted_ur.urdf.xacro",
            "rviz_config_file": PathJoinSubstitution(
                [FindPackageShare("my_robot_workcell"), "rviz", "view_robot.rviz"]
            ),
        }.items()
    )

    # NEW: Spawn the Gripper Motor Controller
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_models_path
        ),
        ur_sim,
        gripper_spawner
    ])