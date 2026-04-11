# Copyright (c) 2021 PickNik, Inc.
# (License text omitted for brevity, keep it in your file if you like!)

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                # KEEP THIS AS "ur_description"! 
                                # We are still using their official launch engine.
                                FindPackageShare("ur_description"),
                                "launch",
                                "view_ur.launch.xml",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "ur_type": LaunchConfiguration("ur_type"),
                    "description_package": "my_robot_workcell",
                    "description_file": "inverted_ur.urdf.xacro",
                    
                    # --- UPDATE THIS LINE ---
                    # Tell it to use the rviz file inside YOUR package
                    "rviz_config_file": PathJoinSubstitution(
                        [FindPackageShare("my_robot_workcell"), "rviz", "view_robot.rviz"]
                    ),
                    # ------------------------
                }.items(),
            )
        ]
    )